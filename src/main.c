/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/lbs.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100

/* The devicetree node identifier for the "ledx" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)
#define SW2_NODE DT_ALIAS(sw2)
#define SW3_NODE DT_ALIAS(sw3)

//define for uart
#define RECEIVE_BUFF_SIZE 10
#define RECEIVE_TIMEOUT 100

//BLE
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define COMPANY_ID_CODE            0x0059

typedef struct adv_mfg_data {
	uint16_t company_code;	    /* Company Identifier Code. */
	uint16_t number_press;      /* Number of times Button 1 is pressed*/
} adv_mfg_data_type;
static adv_mfg_data_type adv_mfg_data = {COMPANY_ID_CODE,0x00};

/*********************/

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led0_t = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1_t = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2_t = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3_t = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

static const struct gpio_dt_spec sw0_t = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec sw1_t = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec sw2_t = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
static const struct gpio_dt_spec sw3_t = GPIO_DT_SPEC_GET(SW3_NODE, gpios);

static struct gpio_callback sw0_cb_data;
static struct gpio_callback sw1_cb_data;
static struct gpio_callback sw2_cb_data;
static struct gpio_callback sw3_cb_data;
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));
static uint8_t tx_buf[] =   {"Press 1-3 on your keyboard to toggle LEDS 1-3 on your development kit\n\r"};
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA,(unsigned char *)&adv_mfg_data, sizeof(adv_mfg_data)),
	
};
// static unsigned char url_data[] = { 0x17, '/', '/', 'a', 'c', 'a', 'd', 'e', 'm',
// 				    'y',  '.', 'n', 'o', 'r', 'd', 'i', 'c', 's',
// 				    'e',  'm', 'i', '.', 'c', 'o', 'm' };

static const struct bt_data sd[] = {
        /* 4.2.3 Include the URL data in the scan response packet*/
		//BT_DATA(BT_DATA_URI, url_data, sizeof(url_data)),
		BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

static struct bt_le_adv_param *adv_param =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY,
	BT_GAP_ADV_FAST_INT_MIN_1,
	BT_GAP_ADV_FAST_INT_MAX_1,
	NULL);
static uint8_t ble_buf_err[] =   {"Bluetooth err\n\r"};
static uint8_t ble_buf_init[] =   {"Bluetooth init\n\r"};
static uint8_t ble_connect[] =   {"Bluetooth connect\n\r"};
static uint8_t ble_disconnect[] =   {"Bluetooth disconnect\n\r"};
static uint8_t ble_led_on[] =   {"LED on\n\r"};
static uint8_t ble_led_off[] =   {"LED off\n\r"};
static uint8_t ble_button[] =   {"Button be touched\n\r"};

struct bt_conn *my_conn = NULL;//connection

static void app_led_cb(bool led_state)
{
	if(led_state)
	{
		gpio_pin_toggle_dt(&led2_t);
		uart_log(uart,ble_led_on, sizeof(ble_led_on),SYS_FOREVER_US);
	}
	else
	{
		gpio_pin_toggle_dt(&led3_t);
		uart_log(uart,ble_led_off, sizeof(ble_led_off),SYS_FOREVER_US);
	}
		
}

static bool app_button_cb(void)
{
	uart_log(uart,ble_button, sizeof(ble_button),SYS_FOREVER_US);
}

static struct bt_lbs_cb app_callbacks = {
	.led_cb = app_led_cb,
	.button_cb = app_button_cb,
};

void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		uart_log(uart, ble_buf_err, sizeof(ble_buf_err), SYS_FOREVER_US);
		return;
	}
	uart_log(uart, ble_connect, sizeof(ble_connect), SYS_FOREVER_US);
	my_conn = bt_conn_ref(conn);

	/* STEP 3.2  Turn the connection status LED on */
	gpio_pin_toggle_dt(&led0_t);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	uart_log(uart, ble_disconnect, sizeof(ble_disconnect), SYS_FOREVER_US);
	bt_conn_unref(my_conn);

	/* STEP 3.3  Turn the connection status LED off */
	gpio_pin_toggle_dt(&led1_t);
}

struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};





void sw0_isr(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void sw1_isr(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void sw2_isr(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void sw3_isr(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);

void wait_ms(int ms)
{
	while(ms>0)
	{
		k_msleep(1);
		ms--;
	}
}
void uart_log(const struct device * dev, const uint8_t * buf, size_t len, int32_t timeout)
{
	uart_tx(dev,buf,len,timeout);
	wait_ms(len);
}

int main(void)
{
	int ret;
	bt_addr_le_t addr;

	/* GPIO configure for led and button */ 
	ret = gpio_pin_configure_dt(&led0_t, GPIO_OUTPUT_ACTIVE);
	ret = gpio_pin_configure_dt(&led1_t, GPIO_OUTPUT_ACTIVE);
	ret = gpio_pin_configure_dt(&led2_t, GPIO_OUTPUT_ACTIVE);
	ret = gpio_pin_configure_dt(&led3_t, GPIO_OUTPUT_ACTIVE);

	ret = gpio_pin_configure_dt(&sw0_t, GPIO_INPUT);
	ret = gpio_pin_configure_dt(&sw1_t, GPIO_INPUT);
	ret = gpio_pin_configure_dt(&sw2_t, GPIO_INPUT);
	ret = gpio_pin_configure_dt(&sw3_t, GPIO_INPUT);

	gpio_pin_interrupt_configure_dt(&sw0_t, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&sw1_t, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&sw2_t, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&sw3_t, GPIO_INT_EDGE_TO_ACTIVE);
	/* ***************************************************************** */

	/* UART set callback function*/
	uart_callback_set(uart, uart_cb, NULL);

	/* Set callback function for button*/
	gpio_init_callback(&sw0_cb_data, sw0_isr, BIT(sw0_t.pin));
	gpio_init_callback(&sw1_cb_data, sw1_isr, BIT(sw1_t.pin));
	gpio_init_callback(&sw2_cb_data, sw2_isr, BIT(sw2_t.pin));
	gpio_init_callback(&sw3_cb_data, sw3_isr, BIT(sw3_t.pin));

	gpio_add_callback(sw0_t.port, &sw0_cb_data);
	gpio_add_callback(sw1_t.port, &sw1_cb_data);
	gpio_add_callback(sw2_t.port, &sw2_cb_data);
	gpio_add_callback(sw3_t.port, &sw3_cb_data);

	/* uart_tx and set uart_rx */
	uart_rx_enable(uart ,rx_buf,sizeof(rx_buf),RECEIVE_TIMEOUT);
	uart_log(uart, tx_buf, sizeof(tx_buf),7000);


	/* BLE */
	ret = bt_addr_le_from_str("FF:EE:DD:CC:BB:AA", "random", &addr);

	bt_conn_cb_register(&connection_callbacks);

	ret = bt_lbs_init(&app_callbacks);
	if (ret) {
		uart_log(uart, ble_buf_err, sizeof(ble_buf_err),SYS_FOREVER_US);
		return;
	}
	
	if(ret)
	{
		uart_log(uart, ble_buf_err, sizeof(ble_buf_err), SYS_FOREVER_US);
	}

	bt_id_create(&addr, NULL);

	ret = bt_enable(NULL);
	if (ret) {
		uart_log(uart, ble_buf_err, sizeof(ble_buf_err), SYS_FOREVER_US);
		return;
	}
	
	uart_log(uart, ble_buf_init, sizeof(ble_buf_init), SYS_FOREVER_US);

	ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret) {
		uart_log(uart, ble_buf_err, sizeof(ble_buf_err), SYS_FOREVER_US);
		return;
	}
	

	return 0;
}


/* callback functions for BUTTON and UART*/
void sw0_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
        gpio_pin_toggle_dt(&led0_t);
}

void sw1_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
        gpio_pin_toggle_dt(&led1_t);
}

void sw2_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
        gpio_pin_toggle_dt(&led2_t);
}

void sw3_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
        gpio_pin_toggle_dt(&led3_t);
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_RX_RDY:
	if((evt->data.rx.len) == 1){
		if(evt->data.rx.buf[evt->data.rx.offset] == '1')
		{	
			gpio_pin_toggle_dt(&led0_t);
			// bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		}
		else if (evt->data.rx.buf[evt->data.rx.offset] == '2')
			gpio_pin_toggle_dt(&led1_t);
		else if (evt->data.rx.buf[evt->data.rx.offset] == '3')
			gpio_pin_toggle_dt(&led2_t);
		else if (evt->data.rx.buf[evt->data.rx.offset] == '4')
			gpio_pin_toggle_dt(&led3_t);
		}
	break;
	case UART_RX_DISABLED:
		uart_rx_enable(dev ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
		break;
	case UART_TX_DONE:
		break;
	default:
		break;
	}
}
