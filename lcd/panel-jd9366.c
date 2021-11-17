// SPDX-License-Identifier: GPL-2.0
/*
 * i.MX drm driver - JD9366 MIPI-DSI panel driver
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#define JD9366_MODE_FLAGS (MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | \
				MIPI_DSI_CLOCK_NON_CONTINUOUS)

/* Manufacturer Command Set pages (CMD2) */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};

static const struct cmd_set_entry manufacturer_cmd_set[] = {

	//Page0
	{0xE0,0x00},

	//--- PASSWORD  ----//
	{0xE1,0x93},
	{0xE2,0x65},
	{0xE3,0xF8},


	//Page0
	{0xE0,0x00},
	//--- Sequence Ctrl  ----//
	{0x70,0x10},	//DC0,DC1
	{0x71,0x13},	//DC2,DC3
	{0x72,0x06},	//DC7
	{0x80,0x03},	//0x03:4-Lane；0x02:3-Lane
	//--- Page4  ----//
	{0xE0,0x04},
	{0x2D,0x03},
	//--- Page1  ----//
	{0xE0,0x01},

	//Set VCOM
	{0x00,0x00},
	{0x01,0xA0},
	//Set VCOM_Reverse
	{0x03,0x00},
	{0x04,0xA0},

	//Set Gamma Power, VGMP,VGMN,VGSP,VGSN
	{0x17,0x00},
	{0x18,0xB1},
	{0x19,0x01},
	{0x1A,0x00},
	{0x1B,0xB1},  //VGMN=0
	{0x1C,0x01},
					
	//Set Gate Power
	{0x1F,0x3E},     //VGH_R  = 15V                       
	{0x20,0x2D},     //VGL_R  = -12V                      
	{0x21,0x2D},     //VGL_R2 = -12V                      
	{0x22,0x0E},     //PA[6]=0, PA[5]=0, PA[4]=0, PA[0]=0 

	//SETPANEL
	{0x37,0x19},	//SS=1,BGR=1

	//SET RGBCYC
	{0x38,0x05},	//JDT=101 zigzag inversion
	{0x39,0x08},	//RGB_N_EQ1, modify 20140806
	{0x3A,0x12},	//RGB_N_EQ2, modify 20140806
	{0x3C,0x78},	//SET EQ3 for TE_H
	{0x3E,0x80},	//SET CHGEN_OFF, modify 20140806 
	{0x3F,0x80},	//SET CHGEN_OFF2, modify 20140806


	//Set TCON
	{0x40,0x06},	//RSO=800 RGB
	{0x41,0xA0},	//LN=640->1280 line

	//--- power voltage  ----//
	{0x55,0x01},	//DCDCM=0001, JD PWR_IC
	{0x56,0x01},
	{0x57,0x69},
	{0x58,0x0A},
	{0x59,0x0A},	//VCL = -2.9V
	{0x5A,0x28},	//VGH = 19V
	{0x5B,0x19},	//VGL = -11V



	//--- Gamma  ----//
	{0x5D,0x7C},              
	{0x5E,0x65},      
	{0x5F,0x53},    
	{0x60,0x48},    
	{0x61,0x43},    
	{0x62,0x35},    
	{0x63,0x39},    
	{0x64,0x23},    
	{0x65,0x3D},    
	{0x66,0x3C},    
	{0x67,0x3D},    
	{0x68,0x5A},    
	{0x69,0x46},    
	{0x6A,0x57},    
	{0x6B,0x4B},    
	{0x6C,0x49},    
	{0x6D,0x2F},    
	{0x6E,0x03},    
	{0x6F,0x00},    
	{0x70,0x7C},    
	{0x71,0x65},    
	{0x72,0x53},    
	{0x73,0x48},    
	{0x74,0x43},    
	{0x75,0x35},    
	{0x76,0x39},    
	{0x77,0x23},    
	{0x78,0x3D},    
	{0x79,0x3C},    
	{0x7A,0x3D},    
	{0x7B,0x5A},    
	{0x7C,0x46},    
	{0x7D,0x57},    
	{0x7E,0x4B},    
	{0x7F,0x49},    
	{0x80,0x2F},    
	{0x81,0x03},    
	{0x82,0x00},    


	//Page2, for GIP
	{0xE0,0x02},

	//GIP_L Pin mapping
	{0x00,0x47},
	{0x01,0x47},  
	{0x02,0x45},
	{0x03,0x45},
	{0x04,0x4B},
	{0x05,0x4B},
	{0x06,0x49},
	{0x07,0x49},
	{0x08,0x41},
	{0x09,0x1F},
	{0x0A,0x1F},
	{0x0B,0x1F},
	{0x0C,0x1F},
	{0x0D,0x1F},
	{0x0E,0x1F},
	{0x0F,0x43},
	{0x10,0x1F},
	{0x11,0x1F},
	{0x12,0x1F},
	{0x13,0x1F},
	{0x14,0x1F},
	{0x15,0x1F},

	//GIP_R Pin mapping
	{0x16,0x46},
	{0x17,0x46},
	{0x18,0x44},
	{0x19,0x44},
	{0x1A,0x4A},
	{0x1B,0x4A},
	{0x1C,0x48},
	{0x1D,0x48},
	{0x1E,0x40},
	{0x1F,0x1F},
	{0x20,0x1F},
	{0x21,0x1F},
	{0x22,0x1F},
	{0x23,0x1F},
	{0x24,0x1F},
	{0x25,0x42},
	{0x26,0x1F},
	{0x27,0x1F},
	{0x28,0x1F},
	{0x29,0x1F},
	{0x2A,0x1F},
	{0x2B,0x1F},
						
	//GIP_L_GS Pin mapping
	{0x2C,0x11},
	{0x2D,0x0F},   
	{0x2E,0x0D}, 
	{0x2F,0x0B}, 
	{0x30,0x09}, 
	{0x31,0x07}, 
	{0x32,0x05}, 
	{0x33,0x18}, 
	{0x34,0x17}, 
	{0x35,0x1F}, 
	{0x36,0x01}, 
	{0x37,0x1F}, 
	{0x38,0x1F}, 
	{0x39,0x1F}, 
	{0x3A,0x1F}, 
	{0x3B,0x1F}, 
	{0x3C,0x1F}, 
	{0x3D,0x1F}, 
	{0x3E,0x1F}, 
	{0x3F,0x13}, 
	{0x40,0x1F}, 
	{0x41,0x1F},
	
	//GIP_R_GS Pin mapping
	{0x42,0x10},
	{0x43,0x0E},   
	{0x44,0x0C}, 
	{0x45,0x0A}, 
	{0x46,0x08}, 
	{0x47,0x06}, 
	{0x48,0x04}, 
	{0x49,0x18}, 
	{0x4A,0x17}, 
	{0x4B,0x1F}, 
	{0x4C,0x00}, 
	{0x4D,0x1F}, 
	{0x4E,0x1F}, 
	{0x4F,0x1F}, 
	{0x50,0x1F}, 
	{0x51,0x1F}, 
	{0x52,0x1F}, 
	{0x53,0x1F}, 
	{0x54,0x1F}, 
	{0x55,0x12}, 
	{0x56,0x1F}, 
	{0x57,0x1F}, 

	//GIP Timing  
	{0x58,0x40},
	{0x59,0x00},
	{0x5A,0x00},
	{0x5B,0x30},
	{0x5C,0x03},
	{0x5D,0x30},
	{0x5E,0x01},
	{0x5F,0x02},
	{0x60,0x00},
	{0x61,0x01},
	{0x62,0x02},
	{0x63,0x03},
	{0x64,0x6B},
	{0x65,0x00},
	{0x66,0x00},
	{0x67,0x73},
	{0x68,0x05},
	{0x69,0x06},
	{0x6A,0x6B},
	{0x6B,0x08},
	{0x6C,0x00},
	{0x6D,0x04},
	{0x6E,0x04},
	{0x6F,0x88},
	{0x70,0x00},
	{0x71,0x00},
	{0x72,0x06},
	{0x73,0x7B},
	{0x74,0x00},
	{0x75,0x07},
	{0x76,0x00},
	{0x77,0x5D},
	{0x78,0x17},
	{0x79,0x1F},
	{0x7A,0x00},
	{0x7B,0x00},
	{0x7C,0x00},
	{0x7D,0x03},
	{0x7E,0x7B},


	//Page1
	{0xE0,0x01},
	{0x0E,0x01},	//LEDON output VCSW2


	//Page3
	{0xE0,0x03},
	{0x98,0x2F},	//From 2E to 2F, LED_VOL

	//Page4
	{0xE0,0x04},
	{0x09,0x10},
	{0x2B,0x2B},
	{0x2E,0x44},

	//Page0
	{0xE0,0x00},
	{0xE6,0x02},
	{0xE7,0x02},

	//SLP OUT
	{0x11},  	// SLPOUT
	//Delayms{120},


	//DISP ON

	{0x29},  	// DSPON
	//Delayms{5},

	//--- TE----//
	{0x35,0x00},

};

static const u32 jd9366_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

struct jd9366_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;
#if 0
	struct gpio_desc *reset;
#else
	int reset;
#endif
#ifdef JD9366_BACKLIGHT
	struct backlight_device *backlight;
#endif

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

static inline struct jd9366_panel *to_jd9366_panel(struct drm_panel *drm_panel)
{
	return container_of(drm_panel, struct jd9366_panel, base);
}

static void jd9366_panel_reset(struct jd9366_panel *disp_panel)
{
	if (disp_panel->reset >= 0) {
		printk(KERN_ERR "jd9366:reseting\n");
		gpio_set_value(disp_panel->reset, 1);
		usleep_range(5000, 10000);
		gpio_set_value(disp_panel->reset, 0);
		usleep_range(5000, 10000);
		gpio_set_value(disp_panel->reset, 1);
		usleep_range(5000, 10000);
	}
}



static int jd9366_panel_push_cmd_list(struct mipi_dsi_device *dsi)
{
	size_t i;
	size_t count = ARRAY_SIZE(manufacturer_cmd_set);
	int ret = 0;
	for (i = 0; i < count; i++) {
		const struct cmd_set_entry *entry = &manufacturer_cmd_set[i];
		u8 buffer[2] =  {entry->cmd, entry->param};  
		ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer));
		if (ret < 0)
			return ret;
	}
	
	return ret;
};

static int jd9366_panel_prepare(struct drm_panel *drm_panel)
{
	
	struct jd9366_panel *disp_panel = to_jd9366_panel(drm_panel);

	if (disp_panel->prepared){
		return 0;
	}
	jd9366_panel_reset(disp_panel);
	disp_panel->prepared = true;

	return 0;
}

static int jd9366_panel_unprepare(struct drm_panel *drm_panel)
{
	
	struct jd9366_panel *disp_panel = to_jd9366_panel(drm_panel);
	struct device *dev = &disp_panel->dsi->dev;

	if (!disp_panel->prepared)
		return 0;

	if (disp_panel->enabled) {
		DRM_DEV_ERROR(dev, "jd9366:Panel still enabled!\n");
		return -EPERM;
	}

	if (disp_panel->reset)
		jd9366_panel_reset(disp_panel);

	disp_panel->prepared = false;

	return 0;
}

static int jd9366_panel_enable(struct drm_panel *drm_panel)
{
	
	struct jd9366_panel *disp_panel = to_jd9366_panel(drm_panel);
	struct mipi_dsi_device *dsi = disp_panel->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (disp_panel->enabled)
		return 0;

	if (!disp_panel->prepared) {
		DRM_DEV_ERROR(dev, "jd9366:Panel not prepared!\n");
		return -EPERM;
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = jd9366_panel_push_cmd_list(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "jd9366:Failed to send MCS (%d)\n", ret);
		goto fail;
	}

	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);		/* 0x01 */
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "jd9366:Failed to do Software Reset (%d)\n", ret);
		goto fail;
	}
	usleep_range(5000, 10000);		/* > 5ms */


	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);	/* 0x11 */
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "jd9366:Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}
	usleep_range(120000, 125000);				/* > 120ms */

	ret = mipi_dsi_dcs_set_display_on(dsi);		/* 0x29 */
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "jd9366:Failed to set display ON (%d)\n", ret);
		goto fail;
	}
	usleep_range(5000, 10000);					/* > 5ms */

	/* Set tear scanline */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);		/* 0x35 */
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "jd9366:Failed to set tear ON (%d)\n", ret);
		goto fail;
	}

#ifdef JD9366_BACKLIGHT
	backlight_enable(disp_panel->backlight);
#endif

	disp_panel->enabled = true;

	return 0;

fail:
	if (disp_panel->reset > 0)
		gpio_set_value(disp_panel->reset, 0);

	return ret;
}

static int jd9366_panel_disable(struct drm_panel *drm_panel)
{
	
	struct jd9366_panel *disp_panel = to_jd9366_panel(drm_panel);
	struct mipi_dsi_device *dsi = disp_panel->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (!disp_panel->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

#ifdef JD9366_BACKLIGHT
	backlight_disable(disp_panel->backlight);
	usleep_range(10000, 12000);
#endif 
	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "jd9366:Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "jd9366:Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	disp_panel->enabled = false;

	return 0;
}


static int jd9366_panel_get_modes(struct drm_panel *drm_panel)
{
	
	struct jd9366_panel *disp_panel = to_jd9366_panel(drm_panel);
	struct device *dev = &disp_panel->dsi->dev;
	struct drm_connector *connector = drm_panel->connector;
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_DEV_ERROR(dev, "jd9366:Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&disp_panel->vm, mode);
	mode->width_mm = disp_panel->width_mm;
	mode->height_mm = disp_panel->height_mm;
	connector->display_info.width_mm = disp_panel->width_mm;
	connector->display_info.height_mm = disp_panel->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	if (disp_panel->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (disp_panel->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (disp_panel->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (disp_panel->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
			jd9366_bus_formats, ARRAY_SIZE(jd9366_bus_formats));
	if (ret)
		return ret;

	drm_mode_probed_add(drm_panel->connector, mode);

	return 1;
}

#ifdef JD9366_BACKLIGHT
static int jd9366_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct jd9366_panel *disp_panel = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	u16 brightness;
	int ret;

	if (!disp_panel->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}


static int jd9366_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct jd9366_panel *disp_panel = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret = 0;

	if (!disp_panel->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "New brightness: %d\n", bl->props.brightness);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops jd9366_bl_ops = {
	.update_status = jd9366_bl_update_status,
	.get_brightness = jd9366_bl_get_brightness,
};
#endif

static const struct drm_panel_funcs jd9366_panel_funcs = {
	.prepare = jd9366_panel_prepare,
	.unprepare = jd9366_panel_unprepare,
	.enable = jd9366_panel_enable,
	.disable = jd9366_panel_disable,
	.get_modes = jd9366_panel_get_modes,
};

static int jd9366_panel_probe(struct mipi_dsi_device *dsi)
{
	
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
	struct jd9366_panel *panel;
#ifdef JD9366_BACKLIGHT
	struct backlight_properties bl_props;
#endif
	int ret;
	u32 video_mode;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;
	panel->prepared = 0;
	panel->enabled = 0;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  JD9366_MODE_FLAGS;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;
		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	/*
	 * 'display-timings' is optional, so verify if the node is present
	 * before calling of_get_videomode so we won't get console error
	 * messages
	 */
	timings = of_get_child_by_name(np, "display-timings");
	if (timings) {
		of_node_put(timings);
		ret = of_get_videomode(np, &panel->vm, 0);
	}
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel-height-mm", &panel->height_mm);

	panel->reset = of_get_named_gpio(np, "reset-gpio", 0);
    if(panel->reset < 0){
        pr_err( "jd9366 gpio not found!\n");
        return -EINVAL;
    }

	ret = gpio_direction_output(panel->reset, 0);
    if(ret != 0){
        pr_err("jd9366 gpio_direction_output fail!\n");
        return -EINVAL;
    }

#ifdef JD9366_BACKLIGHT
	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(
				dev, dev_name(dev),
				dev, dsi,
				&jd9366_bl_ops, &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}
#endif 

	drm_panel_init(&panel->base);
	panel->base.funcs = &jd9366_panel_funcs;
	panel->base.dev = dev;
	dev_set_drvdata(dev, panel);

	ret = drm_panel_add(&panel->base);
	if (ret < 0){
		printk(KERN_ERR "jd9366:drm_panel_add fail\n");
		return ret;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0){
		printk(KERN_ERR "jd9366:mipi_dsi_attach fail\n");
		drm_panel_remove(&panel->base);
	}
	return ret;
}

static int jd9366_panel_remove(struct mipi_dsi_device *dsi)
{
	
	struct jd9366_panel *disp_panel = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "jd9366:Failed to detach from host (%d)\n",
			ret);

	drm_panel_remove(&disp_panel->base);

	return 0;
}

static void jd9366_panel_shutdown(struct mipi_dsi_device *dsi)
{
	
	struct jd9366_panel *disp_panel = mipi_dsi_get_drvdata(dsi);

	jd9366_panel_disable(&disp_panel->base);
	jd9366_panel_unprepare(&disp_panel->base);
}

#ifdef CONFIG_PM
static int jd9366_panel_suspend(struct device *dev)
{
	struct jd9366_panel *disp_panel = dev_get_drvdata(dev);

	if (!disp_panel->reset)
		return 0;

	return 0;
}

static int jd9366_panel_resume(struct device *dev)
{
	struct jd9366_panel *disp_panel = dev_get_drvdata(dev);

	if (disp_panel->reset)
		return 0;

	return 0;
}

#endif

static const struct dev_pm_ops jd9366_pm_ops = {
	SET_RUNTIME_PM_OPS(jd9366_panel_suspend, jd9366_panel_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(jd9366_panel_suspend, jd9366_panel_resume)
};

static const struct of_device_id jd9366_of_match[] = {
	{ .compatible = "jd9366", },
	{ }
};
MODULE_DEVICE_TABLE(of, jd9366_of_match);

static struct mipi_dsi_driver jd9366_panel_driver = {
	.driver = {
		.name = "panel-jd9366",
		.of_match_table = jd9366_of_match,
		.pm	= &jd9366_pm_ops,
	},
	.probe = jd9366_panel_probe,
	.remove = jd9366_panel_remove,
	.shutdown = jd9366_panel_shutdown,
};
module_mipi_dsi_driver(jd9366_panel_driver);

MODULE_AUTHOR("Fenix Lee <leelinfae@163.com>");
MODULE_DESCRIPTION("DRM Driver for JD9366 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
