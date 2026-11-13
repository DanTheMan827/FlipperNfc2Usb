#include "bridge.h"
#include "cp210x_usb.h"
#include "mfrc522.h"
#include "reader.h"

#include <furi.h>
#include <gui/gui.h>
#include <gui/view.h>
#include <gui/view_dispatcher.h>
#include <gui/modules/submenu.h>
#include <gui/modules/variable_item_list.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    AppViewMode,
    AppViewUartStatus,
    AppViewUartSettings,
    AppViewReaderStatus,
} AppView;

typedef enum {
    UsbModeMfrc522Uart,
    UsbModeAcr122u,
    UsbModeAcr1552u,
} UsbMode;

typedef struct {
    BridgeStatus status;
    bool show_usb_log;
} UartModel;

typedef struct {
    ReaderStatus status;
    const char* model;
} ReaderModelView;

typedef struct {
    Gui* gui;
    ViewDispatcher* view_dispatcher;
    Submenu* mode_menu;
    View* uart_view;
    VariableItemList* settings_list;
    View* reader_view;

    Bridge* bridge;
    Reader* reader;
    UsbMode mode;
    bool running;

    BridgeSettings settings;
    uint8_t version_index;
    bool reader_buzzer;
} App;

typedef struct {
    uint8_t value;
    const char* label;
} VersionOption;

static const VersionOption version_options[] = {
    {0x92, "v2.0 92h"},
    {0x91, "v1.0 91h"},
    {0x90, "v0.0 90h"},
    {0x88, "FM17522 88h"},
    {0x12, "Clone 12h"},
};
#define VERSION_OPTION_COUNT (sizeof(version_options) / sizeof(version_options[0]))

enum {
    SettingVersion,
    SettingStrict,
    SettingTiming,
    SettingPowerCycle,
};

/* ------------------------------------------------------------------------ */
/* Engine lifecycle                                                          */
/* ------------------------------------------------------------------------ */

static void app_stop_engine(App* app) {
    if(!app->running) return;
    if(app->bridge) {
        bridge_free(app->bridge);
        app->bridge = NULL;
    }
    if(app->reader) {
        reader_free(app->reader);
        app->reader = NULL;
    }
    app->running = false;
}

static void app_refresh(void* context) {
    App* app = context;
    if(!app->running) return;
    if(app->bridge) {
        BridgeStatus status;
        bridge_get_status(app->bridge, &status);
        with_view_model(app->uart_view, UartModel * model, { model->status = status; }, true);
    } else if(app->reader) {
        ReaderStatus status;
        reader_get_status(app->reader, &status);
        with_view_model(
            app->reader_view, ReaderModelView * model, { model->status = status; }, true);
    }
}

static void app_start_engine(App* app, UsbMode mode) {
    app_stop_engine(app);
    app->mode = mode;
    if(mode == UsbModeMfrc522Uart) {
        app->bridge = bridge_alloc(&app->settings);
        bridge_start(app->bridge);
        app->running = true;
        app_refresh(app);
        view_dispatcher_switch_to_view(app->view_dispatcher, AppViewUartStatus);
    } else {
        ReaderModel model = mode == UsbModeAcr122u ? ReaderModelAcr122u : ReaderModelAcr1552u;
        app->reader = reader_alloc(model);
        reader_set_buzzer(app->reader, app->reader_buzzer);
        reader_start(app->reader);
        app->running = true;
        with_view_model(
            app->reader_view,
            ReaderModelView * m,
            {
                memset(&m->status, 0, sizeof(m->status));
                m->model = reader_model_name(model);
            },
            false);
        app_refresh(app);
        view_dispatcher_switch_to_view(app->view_dispatcher, AppViewReaderStatus);
    }
}

static void app_back_to_menu(App* app) {
    app_stop_engine(app);
    view_dispatcher_switch_to_view(app->view_dispatcher, AppViewMode);
}

/* ------------------------------------------------------------------------ */
/* Mode menu                                                                 */
/* ------------------------------------------------------------------------ */

static void mode_selected(void* context, uint32_t index) {
    App* app = context;
    app_start_engine(app, (UsbMode)index);
}

/* ------------------------------------------------------------------------ */
/* MFRC522 UART status view                                                  */
/* ------------------------------------------------------------------------ */

static const char* cp_request_name(uint8_t r) {
    static const char* const names[] = {
        "IFC_EN",   "SBAUDDIV", "GBAUDDIV", "SLINE",   "GLINE",   "BREAK",
        "IMMCHAR",  "SMHS",     "GMDMSTS",  "XON",     "XOFF",    "SEVMASK",
        "GEVMASK",  "SCHAR",    "GCHARS",   "GPROPS",  "GCOMMST", "RESET",
        "PURGE",    "SFLOW",    "GFLOW",    "EMBED",   "GEVSTATE", NULL,
        NULL,       "SCHARS",   NULL,       NULL,      NULL,      "GBAUD",
        "SBAUD"};
    if(r < sizeof(names) / sizeof(names[0]) && names[r]) return names[r];
    if(r == 0xFF) return "VENDOR";
    return "?";
}

static void uart_draw_usb_log(Canvas* canvas) {
    char line[40];
    Cp210xCtlLog entries[CP210X_LOG_SIZE];
    uint32_t total, stalled;
    cp210x_usb_get_counts(&total, &stalled);
    size_t n = cp210x_usb_get_log(entries, 6);

    canvas_set_font(canvas, FontPrimary);
    snprintf(line, sizeof(line), "USB req %lu stall %lu", (unsigned long)total, (unsigned long)stalled);
    canvas_draw_str(canvas, 0, 9, line);
    canvas_set_font(canvas, FontSecondary);
    for(size_t i = 0; i < n; i++) {
        const Cp210xCtlLog* e = &entries[i];
        snprintf(
            line,
            sizeof(line),
            "%02X %-8s %04X L%u %s",
            e->request,
            cp_request_name(e->request),
            e->value,
            (unsigned)e->length,
            e->stalled ? "STALL" : "ok");
        canvas_draw_str(canvas, 0, (int32_t)(19 + 9 * i), line);
    }
}

static void uart_draw(Canvas* canvas, void* model_ptr) {
    UartModel* model = model_ptr;
    BridgeStatus* s = &model->status;
    char line[48];
    char fmt[8];

    canvas_clear(canvas);
    if(model->show_usb_log) {
        uart_draw_usb_log(canvas);
        return;
    }
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 0, 9, "MFRC522 UART");
    canvas_set_font(canvas, FontSecondary);

    if(!s->usb_ok) {
        canvas_draw_str(canvas, 0, 24, "USB port is busy/locked.");
        canvas_draw_str(canvas, 0, 34, "Close other USB apps");
        canvas_draw_str(canvas, 0, 44, "and try again.");
        return;
    }

    snprintf(
        line,
        sizeof(line),
        "USB %s  Port %s",
        s->usb_configured ? "up" : "--",
        s->port_open ? "open" : "closed");
    canvas_draw_str(canvas, 0, 20, line);

    bridge_format_line(s->line_ctl, fmt, sizeof(fmt));
    snprintf(line, sizeof(line), "Host %lu %s", (unsigned long)s->host_baud, fmt);
    canvas_draw_str(canvas, 0, 30, line);

    snprintf(
        line,
        sizeof(line),
        "Chip %lu (%02X) %s",
        (unsigned long)s->chip_baud,
        s->serial_speed,
        s->over_spec ? "OVERSPD" :
        s->link_ok   ? "ok" :
        s->strict    ? "MISMATCH" :
                       "=host");
    canvas_draw_str(canvas, 0, 40, line);

    snprintf(
        line,
        sizeof(line),
        "RF %s%s %s",
        s->soft_power_down ? "pwrdn" : (s->field_on ? "ON" : "off"),
        s->crypto_on ? " C1" : "",
        mfrc522_command_name(s->command));
    canvas_draw_str(canvas, 0, 50, line);

    canvas_draw_str(canvas, 0, 60, s->last_rf[0] ? s->last_rf : "-");
    canvas_draw_str_aligned(canvas, 128, 0, AlignRight, AlignTop, "OK cfg >log");
}

static bool uart_input(InputEvent* event, void* context) {
    App* app = context;
    if(event->type != InputTypeShort) return false;
    if(event->key == InputKeyOk) {
        view_dispatcher_switch_to_view(app->view_dispatcher, AppViewUartSettings);
        return true;
    }
    if(event->key == InputKeyRight || event->key == InputKeyLeft) {
        with_view_model(
            app->uart_view, UartModel * model, { model->show_usb_log = !model->show_usb_log; }, true);
        return true;
    }
    if(event->key == InputKeyBack) {
        app_back_to_menu(app);
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* Reader status view                                                        */
/* ------------------------------------------------------------------------ */

static void reader_draw(Canvas* canvas, void* model_ptr) {
    ReaderModelView* model = model_ptr;
    ReaderStatus* s = &model->status;
    char line[48];

    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    snprintf(line, sizeof(line), "%s (CCID)", model->model ? model->model : "");
    canvas_draw_str(canvas, 0, 9, line);
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(
        canvas, 128, 0, AlignRight, AlignTop, s->buzzer_enabled ? "OK:mute" : "OK:beep");

    if(!s->usb_ok) {
        canvas_draw_str(canvas, 0, 24, "USB port is busy/locked.");
        canvas_draw_str(canvas, 0, 34, "Close other USB apps");
        canvas_draw_str(canvas, 0, 44, "and try again.");
        return;
    }

    snprintf(
        line,
        sizeof(line),
        "USB %s  RF %s  #%lu",
        s->usb_configured ? "up" : "--",
        s->field_on ? "ON" : "off",
        (unsigned long)s->commands);
    canvas_draw_str(canvas, 0, 20, line);

    snprintf(
        line,
        sizeof(line),
        "Card: %s%s",
        s->card_present ? s->card : "none",
        s->crypto_on ? " C1" : "");
    canvas_draw_str(canvas, 0, 30, line);

    if(s->card_present) {
        snprintf(line, sizeof(line), "UID %s", s->uid);
        canvas_draw_str(canvas, 0, 40, line);
        canvas_draw_str(canvas, 0, 50, s->card_powered ? "Connected (powered)" : "Present");
    } else {
        canvas_draw_str(canvas, 0, 40, "Tap a card on the back");
    }
    canvas_draw_str(canvas, 0, 60, s->last[0] ? s->last : "-");
}

static bool reader_input(InputEvent* event, void* context) {
    App* app = context;
    if(event->type != InputTypeShort) return false;
    if(event->key == InputKeyOk && app->reader) {
        app->reader_buzzer = !app->reader_buzzer;
        reader_set_buzzer(app->reader, app->reader_buzzer);
        return true;
    }
    if(event->key == InputKeyBack) {
        app_back_to_menu(app);
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* MFRC522 settings                                                          */
/* ------------------------------------------------------------------------ */

static void app_push_settings(App* app) {
    if(app->bridge) bridge_set_settings(app->bridge, &app->settings);
}

static void version_changed(VariableItem* item) {
    App* app = variable_item_get_context(item);
    app->version_index = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, version_options[app->version_index].label);
    app->settings.version = version_options[app->version_index].value;
    app_push_settings(app);
}

static void strict_changed(VariableItem* item) {
    App* app = variable_item_get_context(item);
    app->settings.strict_uart = variable_item_get_current_value_index(item) == 1;
    variable_item_set_current_value_text(item, app->settings.strict_uart ? "On" : "Off");
    app_push_settings(app);
}

static void timing_changed(VariableItem* item) {
    App* app = variable_item_get_context(item);
    app->settings.realtime = variable_item_get_current_value_index(item) == 0;
    variable_item_set_current_value_text(item, app->settings.realtime ? "Real" : "Instant");
    app_push_settings(app);
}

static void settings_enter(void* context, uint32_t index) {
    App* app = context;
    if(index == SettingPowerCycle && app->bridge) bridge_power_cycle(app->bridge);
}

static uint32_t settings_previous(void* context) {
    UNUSED(context);
    return AppViewUartStatus;
}

static bool navigation_callback(void* context) {
    UNUSED(context);
    return false; /* back on the mode menu exits */
}

/* ------------------------------------------------------------------------ */

static App* app_alloc(void) {
    App* app = malloc(sizeof(App));
    memset(app, 0, sizeof(App));
    app->settings.version = version_options[0].value;
    app->settings.strict_uart = false;
    app->settings.realtime = true;
    app->reader_buzzer = true;

    app->gui = furi_record_open(RECORD_GUI);
    app->view_dispatcher = view_dispatcher_alloc();
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_navigation_event_callback(app->view_dispatcher, navigation_callback);
    view_dispatcher_set_tick_event_callback(app->view_dispatcher, app_refresh, 250);
    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);

    /* Mode menu */
    app->mode_menu = submenu_alloc();
    submenu_set_header(app->mode_menu, "USB mode");
    submenu_add_item(
        app->mode_menu, "MFRC522 UART (CP2102)", UsbModeMfrc522Uart, mode_selected, app);
    submenu_add_item(app->mode_menu, "ACS ACR122U", UsbModeAcr122u, mode_selected, app);
    submenu_add_item(app->mode_menu, "ACS ACR1552U", UsbModeAcr1552u, mode_selected, app);
    view_dispatcher_add_view(
        app->view_dispatcher, AppViewMode, submenu_get_view(app->mode_menu));

    /* MFRC522 status */
    app->uart_view = view_alloc();
    view_allocate_model(app->uart_view, ViewModelTypeLocking, sizeof(UartModel));
    view_set_context(app->uart_view, app);
    view_set_draw_callback(app->uart_view, uart_draw);
    view_set_input_callback(app->uart_view, uart_input);
    view_dispatcher_add_view(app->view_dispatcher, AppViewUartStatus, app->uart_view);

    /* MFRC522 settings */
    app->settings_list = variable_item_list_alloc();
    VariableItem* item;
    item = variable_item_list_add(
        app->settings_list, "Chip version", VERSION_OPTION_COUNT, version_changed, app);
    variable_item_set_current_value_index(item, app->version_index);
    variable_item_set_current_value_text(item, version_options[app->version_index].label);

    item = variable_item_list_add(app->settings_list, "Strict UART", 2, strict_changed, app);
    variable_item_set_current_value_index(item, 0);
    variable_item_set_current_value_text(item, "Off");

    item = variable_item_list_add(app->settings_list, "Wire timing", 2, timing_changed, app);
    variable_item_set_current_value_index(item, 0);
    variable_item_set_current_value_text(item, "Real");

    item = variable_item_list_add(app->settings_list, "Power-cycle chip", 1, NULL, app);
    variable_item_set_current_value_text(item, "OK");

    variable_item_list_set_enter_callback(app->settings_list, settings_enter, app);
    View* settings_view = variable_item_list_get_view(app->settings_list);
    view_set_previous_callback(settings_view, settings_previous);
    view_dispatcher_add_view(app->view_dispatcher, AppViewUartSettings, settings_view);

    /* Reader status */
    app->reader_view = view_alloc();
    view_allocate_model(app->reader_view, ViewModelTypeLocking, sizeof(ReaderModelView));
    view_set_context(app->reader_view, app);
    view_set_draw_callback(app->reader_view, reader_draw);
    view_set_input_callback(app->reader_view, reader_input);
    view_dispatcher_add_view(app->view_dispatcher, AppViewReaderStatus, app->reader_view);

    return app;
}

static void app_free(App* app) {
    app_stop_engine(app);

    view_dispatcher_remove_view(app->view_dispatcher, AppViewMode);
    view_dispatcher_remove_view(app->view_dispatcher, AppViewUartStatus);
    view_dispatcher_remove_view(app->view_dispatcher, AppViewUartSettings);
    view_dispatcher_remove_view(app->view_dispatcher, AppViewReaderStatus);
    submenu_free(app->mode_menu);
    view_free(app->uart_view);
    variable_item_list_free(app->settings_list);
    view_free(app->reader_view);
    view_dispatcher_free(app->view_dispatcher);
    furi_record_close(RECORD_GUI);
    free(app);
}

int32_t mfrc522_uart_app(void* p) {
    UNUSED(p);
    App* app = app_alloc();
    view_dispatcher_switch_to_view(app->view_dispatcher, AppViewMode);
    view_dispatcher_run(app->view_dispatcher);
    app_free(app);
    return 0;
}
