/********************************************************************
 * ramses_esp
 * wifi_cmd.c
 *
 * (C) 2023 Peter Price
 *
 * WiFi Commands
 *
 */
#include "cmd.h"
#include "wifi_cmd.h"
#include "ramses_wifi.h"

/*********************************************************
 * SSID command
 */
static int wifi_cmd_ssid(int argc, char** argv)
{
    if (argc > 1)
        wifi_set_ssid(argv[1]);

    return 0;
}

/*********************************************************
 * Password command
 */
static int wifi_cmd_password(int argc, char** argv)
{
    if (argc > 1)
        wifi_set_password(argv[1]);

    return 0;
}

/*********************************************************
 * Restart command
 */
static int wifi_cmd_restart(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    wifi_restart();

    return 0;
}

/*********************************************************
 * Status command
 */
static int wifi_cmd_status(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    wifi_status();

    return 0;
}

/*********************************************************
 * Top Level command
 */
static esp_console_cmd_t const wifi_cmds[] = {
    {
        .command = "ssid",
        .help = "Set WiFi <SSID>",
        .hint = NULL,
        .func = &wifi_cmd_ssid,
    },
    {
        .command = "password",
        .help = "Set WiFi <PASSWORD>",
        .hint = NULL,
        .func = &wifi_cmd_password,
    },
    {
        .command = "restart",
        .help = "Restart WiFi",
        .hint = NULL,
        .func = &wifi_cmd_restart,
    },
    {
        .command = "status",
        .help = "Show WiFi status",
        .hint = NULL,
        .func = &wifi_cmd_status,
    },
    { NULL_COMMAND }
};

static int wifi_cmd(int argc, char** argv)
{
    return cmd_menu(argc, argv, wifi_cmds, argv[0]);
}

void wifi_register(void)
{
    const esp_console_cmd_t wifi[] = {
        {
            .command = "wifi",
            .help = "WiFi commands, enter 'wifi' for list",
            .hint = NULL,
            .func = &wifi_cmd,
        },
        { NULL_COMMAND }
    };

    cmd_menu_register(wifi);
}
