/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 12/13/18.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#include <stdio.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libssh/libssh.h>

#include "SshControl.h"

int verify_knownhost(ssh_session session)
{
    enum ssh_known_hosts_e state;
    char buf[10];
    unsigned char *hash = NULL;
    size_t hlen;
    ssh_key srv_pubkey;
    int rc;

    rc = ssh_get_server_publickey(session, &srv_pubkey);
    if (rc < 0) {
        return -1;
    }

    rc = ssh_get_publickey_hash(srv_pubkey,
                                SSH_PUBLICKEY_HASH_SHA256,
                                &hash,
                                &hlen);
    ssh_key_free(srv_pubkey);
    if (rc < 0) {
        return -1;
    }

    state = ssh_session_is_known_server(session);

    switch(state) {
        case SSH_KNOWN_HOSTS_CHANGED:
            fprintf(stderr,"Host key for server changed : server's one is now :\n");
            ssh_print_hash(SSH_PUBLICKEY_HASH_SHA256, hash, hlen);
            ssh_clean_pubkey_hash(&hash);
            fprintf(stderr,"For security reason, connection will be stopped\n");
            return -1;
        case SSH_KNOWN_HOSTS_OTHER:
            fprintf(stderr,"The host key for this server was not found but an other type of key exists.\n");
            fprintf(stderr,"An attacker might change the default server key to confuse your client"
                           "into thinking the key does not exist\n"
                           "We advise you to rerun the client with -d or -r for more safety.\n");
            return -1;
        case SSH_KNOWN_HOSTS_NOT_FOUND:
            fprintf(stderr,"Could not find known host file. If you accept the host key here,\n");
            fprintf(stderr,"the file will be automatically created.\n");
            /* fallback to SSH_SERVER_NOT_KNOWN behavior */
            return -1;
        case SSH_SERVER_NOT_KNOWN:
            fprintf(stderr,
                    "The server is unknown. Do you trust the host key (yes/no)?\n");
            ssh_print_hash(SSH_PUBLICKEY_HASH_SHA256, hash, hlen);

            if (fgets(buf, sizeof(buf), stdin) == NULL) {
                ssh_clean_pubkey_hash(&hash);
                return -1;
            }
            if(strncasecmp(buf,"yes",3)!=0){
                ssh_clean_pubkey_hash(&hash);
                return -1;
            }
            fprintf(stderr,"This new key will be written on disk for further usage. do you agree ?\n");
            if (fgets(buf, sizeof(buf), stdin) == NULL) {
                ssh_clean_pubkey_hash(&hash);
                return -1;
            }
            if(strncasecmp(buf,"yes",3)==0){
                rc = ssh_session_update_known_hosts(session);
                if (rc != SSH_OK) {
                    ssh_clean_pubkey_hash(&hash);
                    fprintf(stderr, "error %s\n", strerror(errno));
                    return -1;
                }
            }

            break;
        case SSH_KNOWN_HOSTS_ERROR:
            ssh_clean_pubkey_hash(&hash);
            fprintf(stderr,"%s",ssh_get_error(session));
            return -1;
        case SSH_KNOWN_HOSTS_OK:
            break; /* ok */
    }

    ssh_clean_pubkey_hash(&hash);

    return 0;
}

int executeSshCmd(const char* cmd) {
    ssh_session session;
    ssh_channel channel;
    char *local_add = "192.168.0.132";
    char *password = "123456";  //or "nvidia"
    char buffer[256];
    int nbytes;
    int rc;

    // Open session and set options
    session = ssh_new();
    ssh_options_set(session, SSH_OPTIONS_HOST, local_add);

    // Connect to server
    rc = ssh_connect(session);
    if (rc != SSH_OK)
    {
        fprintf(stderr, "Error connecting to localhost: %s\n",
                ssh_get_error(session));
        ssh_free(session);
        return -1;
    }

    // Verify the server's identity
    // For the source code of verify_knownhost(), check previous example
//    if (verify_knownhost(session) < 0)
//    {
//        ssh_disconnect(session);
//        ssh_free(session);
//        return -1;
//    }
    rc = ssh_session_update_known_hosts(session);
    if (rc != SSH_OK) {
        fprintf(stderr, "failed to connect.\n");
        return -1;
    }

    // Authenticate ourselves
    rc = ssh_userauth_password(session, NULL, password);
    if (rc != SSH_AUTH_SUCCESS)
    {
        fprintf(stderr, "Error authenticating with password: %s\n", ssh_get_error(session));
        ssh_disconnect(session);
        ssh_free(session);
        return -1;
    }

    // Opening a remote shell
    channel = ssh_channel_new(session);;
    if (channel == NULL) {
        ssh_disconnect(session);
        ssh_free(session);
        ssh_finalize();
        return 1;
    }

    rc = ssh_channel_open_session(channel);
    if (rc < 0) {
        goto failed;
    }

    // Passing a remote command here
    rc = ssh_channel_request_exec(channel, cmd);
    if (rc < 0) {
        goto failed;
    }

    // Displaying the data sent by the remote computer
    nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
    while (nbytes > 0) {
        if (fwrite(buffer, 1, nbytes, stdout) != (unsigned int) nbytes) {
            goto failed;
        }
        nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
    }

    if (nbytes < 0) {
        goto failed;
    }

    // Clean up
    ssh_channel_send_eof(channel);
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    ssh_disconnect(session);
    ssh_free(session);
    ssh_finalize();

    return 0;
    failed:
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    ssh_disconnect(session);
    ssh_free(session);
    ssh_finalize();

    return 1;
}