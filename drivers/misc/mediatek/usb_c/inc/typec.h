/*
 * This is used to for host and peripheral modes of the tyoec driver.
 */

#ifndef USB_TYPEC_H
#define	USB_TYPEC_H

/* ConnSide */
#define DONT_CARE	0
#define UP_SIDE		1
#define DOWN_SIDE	2

/* Stat */
#define	DISABLE		0
#define	ENABLE		1

/* DriverType */
#define DEVICE_TYPE	1
#define HOST_TYPE	2

/* USBRdCtrlPin */
#define U3_EQ_C1	0
#define U3_EQ_C2	1

/* USBRdStat */
#define U3_EQ_LOW	0
#define U3_EQ_HZ	1
#define U3_EQ_HIGH	2

#ifdef CONFIG_USB_C_SWITCH_FUSB302
struct usb3_switch {
	int sel_gpio;
	int en_gpio;
	int sel;
	int en;
};
#endif

struct usb_redriver {
	int c1_gpio;
	int c2_gpio;
	int eq_c1;
	int eq_c2;
};

struct typec_switch_data {
	char *name;
	int type;
	int on;
	int (*enable)(void *);
	int (*disable)(void *);
	void *priv_data;
};

#ifdef CONFIG_USB_C_SWITCH_FUSB302
struct usbc_pin_ctrl {
	struct pinctrl_state *fusb340_sel_init;
	struct pinctrl_state *fusb340_sel_low;
	struct pinctrl_state *fusb340_sel_high;

	struct pinctrl_state *fusb340_usb18_init;
	struct pinctrl_state *fusb340_usb18_low;
	struct pinctrl_state *fusb340_usb18_high;
};
#elif defined(CONFIG_USB_C_SWITCH_SII70XX)
struct usbc_pin_ctrl {
	struct pinctrl_state *re_c1_init;
	struct pinctrl_state *re_c2_init;
	struct pinctrl_state *sii7033_rst_init;
	struct pinctrl_state *sii7033_rst_low;
	struct pinctrl_state *sii7033_rst_high;
};
#elif defined(CONFIG_USB_C_SWITCH_ANX7418)
struct usbc_pin_ctrl {
	struct pinctrl_state *rst_n_init;
	struct pinctrl_state *rst_n_low;
	struct pinctrl_state *rst_n_high;

	struct pinctrl_state *pwr_en_init;
	struct pinctrl_state *pwr_en_low;
	struct pinctrl_state *pwr_en_high;

	struct pinctrl_state *cbl_det_init;
	struct pinctrl_state *intp_init;
};
#endif

/*
 * struct usbtypc - Driver instance data.
 */
/*#ifdef CONFIG_USB_C_SWITCH_FUSB302  delete by cassy*/
/*add by cassy begin*/
#ifdef CONFIG_USB_C_SWITCH_FUSB302
struct usbtypc {
	int irqnum;
	int en_irq;
#ifdef CONFIG_MTK_SIB_USB_SWITCH
	bool sib_enable;
#endif
	struct pinctrl *pinctrl;
	struct usbc_pin_ctrl *pin_cfg;
	spinlock_t	fsm_lock;
	struct delayed_work fsm_work;
	struct i2c_client *i2c_hd;
	struct hrtimer toggle_timer;
	struct hrtimer debounce_timer;
	struct typec_switch_data *host_driver;
	struct typec_switch_data *device_driver;
	struct usb3_switch *u3_sw;
	struct usb_redriver *u_rd;
};
#elif defined(CONFIG_USB_C_SWITCH_SII70XX)
struct usbtypc {
	int irqnum;
	int en_irq;
	struct pinctrl *pinctrl;
	struct usbc_pin_ctrl *pin_cfg;
	struct delayed_work eint_work;
	struct i2c_client *i2c_hd;
	struct typec_switch_data *host_driver;
	struct typec_switch_data *device_driver;
	struct usb_redriver *u_rd;
};
#elif defined(CONFIG_USB_C_SWITCH_ANX7418)
struct usbtypc {
	struct pinctrl *pinctrl;
	struct usbc_pin_ctrl *pin_cfg;
	struct device *pinctrl_dev;
	struct i2c_client *i2c_hd;
	struct typec_switch_data *host_driver;
	struct typec_switch_data *device_driver;
};
#endif

struct usb_typc_driver {
#ifdef CONFIG_MTK_SIB_USB_SWITCH
	bool sib_enable;
#endif
	struct pinctrl *pinctrl;
	struct pinctrl_state *int_cfg;
	struct pinctrl_state *en_low;
	struct pinctrl_state *en_high;

	unsigned int conn_state;
	struct typec_switch_data *host_driver;
	struct typec_switch_data *device_driver;	
};

extern int trigger_driver(int type, int stat, int dir);
extern int register_typec_switch_callback(struct typec_switch_data *new_driver);
extern int unregister_typec_switch_callback(struct typec_switch_data *new_driver);

#if defined(CONFIG_USB_C_SWITCH_SII70XX) ||defined(CONFIG_USB_C_SWITCH_FUSB302)
extern int usb_redriver_config(struct usbtypc *typec, int ctrl_pin, int stat);
extern int usb_redriver_enter_dps(struct usbtypc *typec);
extern int usb_redriver_exit_dps(struct usbtypc *typec);
#endif
#endif	/* USB_TYPEC_H */
