#include "json_rpc_client.h"

#define MAX_RESPONSE_BUF_SZ (1024 * 8)

AmbientLightingDaemon daemon_flavor = DAEMON_NOT_SET;

const char* daemon_to_string(AmbientLightingDaemon flavor)
{
    switch (flavor) {
    case DAEMON_INVALID:
        return "INVALID";
    case DAEMON_NOT_SET:
        return "NOT SET";
    case DAEMON_HYPERION_NG:
        return "Hyperion.NG";
    case DAEMON_HYPERHDR:
        return "HyperHDR";
    default:
        return "<UNKNOWN>";
    }
}

int do_http_post(char* url, const char* post_body, char** response_body, int out_buf_sz)
{
    int ret = 0;

    char* command = (char*)calloc(1, PATH_MAX);
    if (command == NULL) {
        ERR("do_http_post: alloc failed -> command");
        ret = -1;
        goto exit;
    }

    sprintf(command, "curl --silent -X POST %s -d '%s'", url, post_body);
    DBG("do_http_post: Command: %s", command);

    FILE* fd = popen(command, "r");
    if (fd == NULL) {
        ERR("popen failed, command: %s!", command);
        ret = -2;
        goto exit;
    }

    fread(*response_body, out_buf_sz, 1, fd);

    ret = pclose(fd);
    if (ret != 0) {
        ERR("Curl process failed! Code: 0x%x", ret);
        ret = -3;
        goto exit;
    }

exit:
    if (command != NULL)
        free(command);

    return ret;
}

int send_rpc_message(char* host, ushort rpc_port, jvalue_ref post_body_jval, jvalue_ref* response_body_jval)
{
    int ret = 0;
    bool command_success = false;
    jvalue_ref success_field;
    JSchemaInfo schema;

    char* url = (char*)calloc(1, PATH_MAX);
    char* response = (char*)calloc(1, MAX_RESPONSE_BUF_SZ);

    if (url == NULL) {
        ERR("send_rpc_message: alloc failed -> url");
        ret = -1;
        goto exit;
    }

    if (response == NULL) {
        ERR("send_rpc_message: alloc failed -> response");
        ret = -2;
        goto exit;
    }

    sprintf(url, "http://%s:%d/json-rpc", host, rpc_port);
    DBG("JSON-RPC target: %s", url);

    const char* post_body = jvalue_tostring_simple(post_body_jval);

    if ((ret = do_http_post(url, post_body, &response, MAX_RESPONSE_BUF_SZ)) != 0) {
        WARN("send_rpc_message: HTTP POST request failed, ret: %d", ret);
        ret = -3;
        goto exit;
    }

    jschema_info_init(&schema, jschema_all(), NULL, NULL);
    *response_body_jval = jdom_parse(j_cstr_to_buffer(response), DOMOPT_NOOPT, &schema);

    // Parsing failed
    if (!jis_valid(*response_body_jval) || jis_null(*response_body_jval)) {
        ERR("Failed to parse RPC response from %s:%d", host, rpc_port);
        ret = -4;
        goto exit;
    }

    // Check boolean field "success" in response
    success_field = jobject_get(*response_body_jval, j_cstr_to_buffer("success"));
    if (!jis_boolean(success_field) || jboolean_get(success_field, &command_success) || !command_success) {
        ERR("Invalid success status from %s:%d", host, rpc_port);
        ret = -5;
        goto exit;
    }

exit:
    if (response != NULL)
        free(response);
    if (url != NULL)
        free(url);

    return ret;
}

int get_daemon_flavor(char* host, ushort rpc_port, AmbientLightingDaemon* flavor)
{
    int ret = 0;

    jvalue_ref info_node;
    jvalue_ref daemon_node;
    jvalue_ref response_body_jval;
    jvalue_ref post_body = jobject_create();
    jobject_set(post_body, j_cstr_to_buffer("command"), jstring_create("sysinfo"));

    if ((ret = send_rpc_message(host, rpc_port, post_body, &response_body_jval)) != 0) {
        WARN("get_daemon_flavor: Failed to send RPC message, code: %d", ret);
        j_release(&post_body);
        return -1;
    }

    /*
     * Response difference between Hyperion.NG and HyperHDR
     * - Hyperion.NG: { "command" "info": { "hyperion" : {...} }, "success": true }
     * - HyperHDR: { "command" "info": { "hyperhdr" : {...} }, "success": true }
     */
    if ((info_node = jobject_get(response_body_jval, j_cstr_to_buffer("info"))) && jis_valid(info_node)) {
        if ((daemon_node = jobject_get(info_node, j_cstr_to_buffer("hyperion"))) && jis_valid(daemon_node)) {
            DBG("get_daemon_flavor: Detected Hyperion.NG");
            *flavor = DAEMON_HYPERION_NG;
        } else if ((daemon_node = jobject_get(info_node, j_cstr_to_buffer("hyperhdr"))) && jis_valid(daemon_node)) {
            DBG("get_daemon_flavor: Detected HyperHDR");
            *flavor = DAEMON_HYPERHDR;
        } else {
            WARN("get_daemon_flavor: Unknown daemon flavor!");
            *flavor = DAEMON_INVALID;
            ret = -2;
        }
    } else {
        WARN("Failed to fetch daemon flavor from JSON-RPC, no 'info' node");
        ret = -3;
    }

    j_release(&post_body);
    return ret;
}

int set_hdr_state(char* host, ushort rpc_port, bool hdr_active)
{
    int ret = 0;

    if (daemon_flavor == DAEMON_NOT_SET || daemon_flavor == DAEMON_INVALID) {
        DBG("set_hdr_state: Currently known daemon flavor: %d (%s) -> Fetching new state",
            daemon_flavor, daemon_to_string(daemon_flavor));
        if ((ret = get_daemon_flavor(host, rpc_port, &daemon_flavor)) != 0) {
            ERR("set_hdr_state: Failed to fetch daemon flavor, ret: %d", ret);
            return -1;
        }
        INFO("Detected daemon flavor: %d (%s)", daemon_flavor, daemon_to_string(daemon_flavor));
    }

    if (daemon_flavor != DAEMON_HYPERHDR) {
        WARN("set_hdr_state: Daemon is not HyperHDR -> Not submitting HDR state!");
        return -2;
    }

    jvalue_ref response_body_jval;
    jvalue_ref post_body = jobject_create();
    jvalue_ref component_state_jobj = jobject_create();

    // Assemble nested object first
    // See: https://docs.hyperion-project.org/en/json/Control.html#control-components
    jobject_set(component_state_jobj, j_cstr_to_buffer("component"), jstring_create("HDR"));
    jobject_set(component_state_jobj, j_cstr_to_buffer("state"), jboolean_create(hdr_active));

    // Assemble top-level json
    jobject_set(post_body, j_cstr_to_buffer("command"), jstring_create("componentstate"));
    jobject_set(post_body, j_cstr_to_buffer("componentstate"), component_state_jobj);

    if ((ret = send_rpc_message(host, rpc_port, post_body, &response_body_jval)) != 0) {
        WARN("set_hdr_state: Failed to send RPC message, code: %d", ret);
        ret = -3;
    }

    j_release(&post_body);
    j_release(&component_state_jobj);
    return ret;
}

int set_hdr_mode(char* host, ushort rpc_port, const char* hdr_type)
{
    int ret = 0;
    const char* lut_filename = NULL;
    int hdr_enabled = 0;

    if (strcmp(hdr_type, "none") == 0) {
        hdr_enabled = 0;
    } else {
        hdr_enabled = 1;
        if (strcmp(hdr_type, "DolbyVision") == 0) {
            lut_filename = "lut_lin_tables_dv.3d";
        } else {
            lut_filename = "lut_lin_tables_hdr.3d";
        }
    }

    if (daemon_flavor == DAEMON_NOT_SET || daemon_flavor == DAEMON_INVALID) {
        DBG("set_hdr_mode: Currently known daemon flavor: %d (%s) -> Fetching new state",
            daemon_flavor, daemon_to_string(daemon_flavor));
        if ((ret = get_daemon_flavor(host, rpc_port, &daemon_flavor)) != 0) {
            ERR("set_hdr_mode: Failed to fetch daemon flavor, ret: %d", ret);
            return -1;
        }
        INFO("Detected daemon flavor: %d (%s)", daemon_flavor, daemon_to_string(daemon_flavor));
    }

    if (daemon_flavor != DAEMON_HYPERHDR) {
        WARN("set_hdr_mode: Daemon is not HyperHDR -> Not submitting HDR mode!");
        return -2;
    }

    jvalue_ref response_body_jval;
    jvalue_ref post_body = jobject_create();

    jobject_set(post_body, j_cstr_to_buffer("command"), jstring_create("videomodehdr"));
    jobject_set(post_body, j_cstr_to_buffer("HDR"), jnumber_create_i32(hdr_enabled));
    if (lut_filename) {
        jobject_set(post_body, j_cstr_to_buffer("flatbuffers_user_lut_filename"), jstring_create(lut_filename));
    }

    if ((ret = send_rpc_message(host, rpc_port, post_body, &response_body_jval)) != 0) {
        WARN("set_hdr_mode: Failed to send RPC message, code: %d", ret);
        ret = -3;
    }

    j_release(&post_body);
    return ret;
}