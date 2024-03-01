/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
// #include <bluetooth/services/lbs.h>

#include <bluetooth/services/nus.h>
/* STEP 1.2 - Add the header file for the Settings module */
#include <zephyr/settings/settings.h>

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

#define CMD_TEST		'A'

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
		//BT_DATA(BT_DATA_URI, url_data, sizeof(url_data)),
		// BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
		BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static struct bt_le_adv_param *adv_param =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY,
	BT_GAP_ADV_FAST_INT_MIN_1,
	BT_GAP_ADV_FAST_INT_MAX_1,
	NULL);

static int LED_STATUS[4] = {1,1,1,1};

struct bt_conn *my_conn = NULL;//connection

void led_blinky()
{
	uint8_t loop;
	printk("Led blinky!!!!!!!!!!!!!\r\n");
	for(loop=0;loop<5;loop++)
	{
		gpio_pin_set_dt(&led0_t, 1);
		gpio_pin_set_dt(&led1_t, 1);
		gpio_pin_set_dt(&led2_t, 1);
		gpio_pin_set_dt(&led3_t, 1);
		k_msleep(300);
		gpio_pin_set_dt(&led0_t, 0);
		gpio_pin_set_dt(&led1_t, 0);
		gpio_pin_set_dt(&led2_t, 0);
		gpio_pin_set_dt(&led3_t, 0);
		k_msleep(300);

	}
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	int err;int i;
	char addr[BT_ADDR_LE_STR_LEN] = { 0 };
	uint8_t data_decode[32]={NULL,};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	gpio_pin_toggle_dt(&led2_t);
	printk("Received data from: %s", addr);
	
	for(i=0;i<len;i++)
	{
		data_decode[i] = *(data+i);
	}
	printk(data_decode);
	if(data_decode[0] == 'A')
		{
			led_blinky();
		}

}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("ble err!!!!!!");
		return;
	}
	printk("BLE connected");
	my_conn = bt_conn_ref(conn);

	/* STEP 3.2  Turn the connection status LED on */
	gpio_pin_toggle_dt(&led0_t);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("BLE Disconnect!!!!\r\n");
	bt_conn_unref(my_conn);

	/* STEP 3.3  Turn the connection status LED off */
	gpio_pin_toggle_dt(&led1_t);
}
static void on_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level, err);
	}
}

struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
	.security_changed = on_security_changed,
};

void sw0_isr(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void sw1_isr(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void sw2_isr(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void sw3_isr(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);


/*************************************************/
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}
static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};
/********************************************************/

int main(void)
{
	int ret,err;
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
	printk(tx_buf);


	/* BLE */
	ret = bt_addr_le_from_str("FF:EE:DD:CC:BB:AA", "random", &addr);

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		printk("Failed to register authorization callbacks.\n");
		return;
	}

	bt_conn_cb_register(&connection_callbacks);

	ret = bt_nus_init(&nus_cb);
	if (ret) {
		printk("BLE ERR !!!!!");
		return;
	}
	
	if(ret)
	{
		printk("BLE ERR !!!!!");
	}

	bt_id_create(&addr, NULL);

	ret = bt_enable(NULL);
	if (ret) {
		printk("BLE ERR !!!!!");
		return;
	}
	settings_load();
	
	printk("BLE init done !!!!!");

	ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret) {
		printk("BLE ERR !!!!!");
		return;
	}
	

	return 0;
}

void led_control(int led_stt)
{
	if(LED_STATUS[led_stt]==1)
	{
		LED_STATUS[led_stt]=0;
	}
	else
		LED_STATUS[led_stt]=1;

	switch (led_stt)
	{
	case 0:
		gpio_pin_set_dt(&led0_t,LED_STATUS[led_stt]);
		break;
	case 1:
		gpio_pin_set_dt(&led1_t,LED_STATUS[led_stt]);
		break;
	case 2:
		gpio_pin_set_dt(&led2_t,LED_STATUS[led_stt]);
		break;
	case 3:
		gpio_pin_set_dt(&led3_t,LED_STATUS[led_stt]);
		break;
	
	default:
		break;
	}
}
/* callback functions for BUTTON and UART*/
void sw0_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
		uint8_t nus_tx[5]={'s','w','0',':','1'};
		int value;
        led_control(0);
		if(LED_STATUS[0])
			nus_tx[4] = '1';
		else
			nus_tx[4]='0';
		bt_nus_send(my_conn,nus_tx,5);
}

void sw1_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
		uint8_t nus_tx[5]={'s','w','1',':','1'};
		int value;
        led_control(1);
		if(LED_STATUS[1])
			nus_tx[4] = '1';
		else
			nus_tx[4]='0';
		bt_nus_send(my_conn,nus_tx,5);
}

void sw2_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
		uint8_t nus_tx[5]={'s','w','2',':','1'};
		int value;
        led_control(2);
		if(LED_STATUS[2])
			nus_tx[4] = '1';
		else
			nus_tx[4]='0';
		bt_nus_send(my_conn,nus_tx,5);
}

void sw3_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
		uint8_t nus_tx[5]={'s','w','3',':','1'};
		int value;
        led_control(3);
		if(LED_STATUS[3])
			nus_tx[4] = '1';
		else
			nus_tx[4]='0';
		bt_nus_send(my_conn,nus_tx,5);
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_RX_RDY:
	if((evt->data.rx.len) == 1){
		// if(evt->data.rx.buf[evt->data.rx.offset] == '1')
		// {	
		// 	gpio_pin_toggle_dt(&led0_t);
		// }
		// else if (evt->data.rx.buf[evt->data.rx.offset] == '2')
		// 	gpio_pin_toggle_dt(&led1_t);
		// else if (evt->data.rx.buf[evt->data.rx.offset] == '3')
		// 	gpio_pin_toggle_dt(&led2_t);
		// else if (evt->data.rx.buf[evt->data.rx.offset] == '4')
		// 	gpio_pin_toggle_dt(&led3_t);
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

