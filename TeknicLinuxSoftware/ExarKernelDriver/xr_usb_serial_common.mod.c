#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xd9726f80, "module_layout" },
	{ 0x99a6bbcb, "usb_deregister" },
	{ 0xc5850110, "printk" },
	{ 0xd1f2225e, "put_tty_driver" },
	{ 0x6591c4d8, "tty_unregister_driver" },
	{ 0x4fe357bb, "usb_register_driver" },
	{ 0x851f1bf9, "tty_register_driver" },
	{ 0xfeeb5df9, "tty_set_operations" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0xd153a723, "__tty_alloc_driver" },
	{ 0x7e7a175d, "tty_port_register_device" },
	{ 0xe71fddd1, "usb_get_intf" },
	{ 0x12adac2e, "usb_driver_claim_interface" },
	{ 0xc6810313, "_dev_info" },
	{ 0xac133870, "_dev_warn" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x8c29242a, "device_create_file" },
	{ 0x8a68ac56, "usb_alloc_urb" },
	{ 0x6c2b84b3, "usb_alloc_coherent" },
	{ 0x7f0259dd, "tty_port_init" },
	{ 0x977f511b, "__mutex_init" },
	{ 0x12947aab, "usb_ifnum_to_if" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xb2fd5ceb, "__put_user_4" },
	{ 0x69ecc112, "kmem_cache_alloc_trace" },
	{ 0x36c11c94, "kmalloc_caches" },
	{ 0xc6cbbc89, "capable" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x6729d3df, "__get_user_4" },
	{ 0x409873e3, "tty_termios_baud_rate" },
	{ 0xed537d8, "tty_port_open" },
	{ 0xe0a537d, "usb_autopm_put_interface" },
	{ 0x94f2bc5d, "usb_autopm_get_interface" },
	{ 0xc959d152, "__stack_chk_fail" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x50d4de4, "pv_ops" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0xac847abb, "tty_flip_buffer_push" },
	{ 0x57c928a1, "tty_insert_flip_string_fixed_flag" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x52fa0526, "tty_standard_install" },
	{ 0x766109d0, "usb_driver_release_interface" },
	{ 0x8971e83c, "usb_free_urb" },
	{ 0x39f96a41, "tty_unregister_device" },
	{ 0x71180e7, "tty_kref_put" },
	{ 0xa9456c46, "tty_vhangup" },
	{ 0xb80f7763, "tty_port_tty_get" },
	{ 0xa92c69c7, "device_remove_file" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x332e474f, "usb_kill_urb" },
	{ 0x1fe0f19d, "tty_port_close" },
	{ 0x9bba2e06, "usb_autopm_get_interface_async" },
	{ 0x4066fa5b, "tty_port_hangup" },
	{ 0xcac3c27c, "tty_port_tty_wakeup" },
	{ 0x37a0cba, "kfree" },
	{ 0x110edbe1, "usb_put_intf" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xb006406c, "tty_port_tty_hangup" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x3f444986, "tty_port_put" },
	{ 0x8d1c6da6, "usb_free_coherent" },
	{ 0x85c47850, "usb_autopm_put_interface_async" },
	{ 0x1a1edd5a, "_dev_err" },
	{ 0xff3a7e30, "usb_submit_urb" },
	{ 0x69acdf38, "memcpy" },
	{ 0x327d1f7c, "__dynamic_dev_dbg" },
	{ 0x46173060, "usb_control_msg" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v04E2p1410d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1411d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1412d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1414d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1420d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1421d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1422d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1424d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1400d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1401d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1402d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1403d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v2890p0213d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "B2542D718D56BDE08DB7088");
