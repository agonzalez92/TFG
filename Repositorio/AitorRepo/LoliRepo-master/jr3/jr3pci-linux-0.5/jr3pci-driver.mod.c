#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xae141548, "module_layout" },
	{ 0xf2a644fb, "copy_from_user" },
	{ 0x2da418b5, "copy_to_user" },
	{ 0xe8b8056d, "__register_chrdev" },
	{ 0x3af98f9e, "ioremap_nocache" },
	{ 0xadf42bd5, "__request_region" },
	{ 0xb8aa2342, "__check_region" },
	{ 0x2b3acc9, "pci_enable_device" },
	{ 0xace06376, "pci_get_device" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x3c2c5af5, "sprintf" },
	{ 0xb6ed1e53, "strncpy" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x9bce482f, "__release_region" },
	{ 0x788fe103, "iomem_resource" },
	{ 0xedc03953, "iounmap" },
	{ 0xb72397d5, "printk" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

