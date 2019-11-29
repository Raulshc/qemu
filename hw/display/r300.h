/*
 * QEMU R300 SVGA emulation
 *
 * Copyright (c) 2019 Aaron Zakhrov
 *
 * This work is licensed under the GNU GPL license version 2 or later.
 */



#ifndef R300_H
#define R300_H

#include "qemu/timer.h"
#include "hw/pci/pci.h"
#include "hw/i2c/bitbang_i2c.h"
#include "vga_int.h"

/*#define DEBUG_R300*/

#define CRTC_PIX_WIDTH_4BPP                     0x00000100
#define CRTC_PIX_WIDTH_8BPP                     0x00000200
#define CRTC_PIX_WIDTH_15BPP                    0x00000300
#define CRTC_PIX_WIDTH_16BPP                    0x00000400
#define CRTC_PIX_WIDTH_24BPP                    0x00000500
#define CRTC_PIX_WIDTH_32BPP                    0x00000600


#define PCI_VENDOR_ID_ATI 0x1002
/* Radeon 9500 PRO */
#define PCI_DEVICE_ID_ATI_RADEON_9500_PRO 0x4e45
/* Radeon 9500 PRO */
#define PCI_DEVICE_ID_ATI_RADEON_9700 0x4e44

#define PCI_DEVICE_ID_ATI_RADEON_X550 0x5B60
#define PCI_DEVICE_ID_ATI_RADEON_X1600_PRO 0x71C2

#define RADEON_MIN_MMIO_SIZE 0x10000


#define TYPE_RAD_VGA "rad-vga"
#define RAD_VGA(obj) OBJECT_CHECK(RADVGAState, (obj), TYPE_RAD_VGA)

typedef struct RADVGARegs{

  uint64_t mm_index;
  uint64_t bios_scratch[8];
  uint64_t gen_int_cntl;
  uint64_t gen_int_status;
  uint64_t crtc_gen_cntl;
  uint64_t crtc_ext_cntl;
  uint64_t dac_cntl;
  uint64_t gpio_vga_ddc;
  uint64_t gpio_dvi_ddc;
  uint64_t gpio_monid;
  uint64_t config_cntl;
  uint64_t crtc_h_total_disp;
  uint64_t crtc_h_sync_strt_wid;
  uint64_t crtc_v_total_disp;
  uint64_t crtc_v_sync_strt_wid;
  uint64_t crtc_offset;
  uint64_t crtc_offset_cntl;
  uint64_t crtc_pitch;
  uint64_t cur_offset;
  uint64_t cur_hv_pos;
  uint64_t cur_hv_offs;
  uint64_t cur_color0;
  uint64_t cur_color1;
  uint64_t dst_offset;
  uint64_t dst_pitch;
  uint64_t dst_tile;
  uint64_t dst_width;
  uint64_t dst_height;
  uint64_t src_offset;
  uint64_t src_pitch;
  uint64_t src_tile;
  uint64_t src_x;
  uint64_t src_y;
  uint64_t dst_x;
  uint64_t dst_y;
  uint64_t dp_gui_master_cntl;
  uint64_t dp_brush_bkgd_clr;
  uint64_t dp_brush_frgd_clr;
  uint64_t dp_src_frgd_clr;
  uint64_t dp_src_bkgd_clr;
  uint64_t dp_cntl;
  uint64_t dp_datatype;
  uint64_t dp_mix;
  uint64_t dp_write_mask;
  uint64_t default_offset;
  uint64_t default_pitch;
  uint64_t default_tile;
  uint64_t default_sc_bottom_right;
  uint64_t mc_status;
  uint64_t isync_cntl;
  uint64_t host_path_cntl;
  uint64_t wait_until;
  uint64_t cp_csq_cntl;
  uint64_t scratch_umask;
  uint64_t r100_display_base_addr;
  uint64_t r100_sclk_cntl;
  uint64_t pcie_index;
  uint64_t pcie_data;
    uint64_t aic_lo_addr;
      uint64_t aic_hi_addr;
      uint64_t fp_gen_cntl;
      uint64_t mm_data;



  uint8_t vga_reset;
  uint64_t tile_x0_y0;
  uint64_t dda_config;
  uint64_t aic_cntl;

  uint64_t cp_rb_cntl;
  uint64_t mem_cntl;


  uint64_t surface_cntl;
  uint64_t surface0_info;
  uint64_t surface1_info;
  uint64_t surface2_info;
  uint64_t surface3_info;
  uint64_t surface4_info;
  uint64_t surface5_info;
  uint64_t surface6_info;
  uint64_t surface7_info;
  uint64_t ov0_scale_cntl;
  uint64_t i2c_cntl_1;
  uint64_t dvi_i2c_cntl_1;
  uint64_t subpic_cntl;
  uint64_t viph_control;
  uint64_t cap0_trig_cntl;
  uint64_t cap1_trig_cntl;
  uint64_t cur2_offset;

  uint64_t crtc2_gen_cntl;

  uint64_t mem_intf_cntl;
  uint64_t agp_base_2;
  uint64_t agp_base;

  uint64_t mem_addr_config;
  uint64_t display2_base_addr;
  uint64_t spll_cntl;
  uint64_t vclk_ecp_cntl;

  uint64_t aic_pt_base;
  uint64_t pci_gart_page;
  uint64_t mc_agp_location;






  //R300 DST registers

  uint64_t r300_dst_pipe_config;

  //R300 GB Registers
  uint64_t r300_gb_enable;
  uint64_t r300_gb_tile_config;
  uint64_t r300_gb_fifo_size;
  uint64_t r300_gb_select;
  uint64_t r300_gb_aa_config;
  uint64_t r300_gb_mpos_0;
  uint64_t r300_gb_mpos_1;

  // RE registers
  uint64_t r300_re_scissors_tl;
  uint64_t r300_re_scissors_br;

  // RB2D registers
uint64_t r300_rb2d_dstcache_mode;

// RB3D Registers
  uint64_t r300_rb3d_aaresolve_ctl;
  uint64_t r300_rb3d_aaresolve_offset;
  uint64_t r300_rb3d_aaresolve_pitch;
  uint64_t r300_rb3d_ablend;
  uint64_t r300_rb3d_blend_color;
  uint64_t r300_rb3d_cblend;
  uint64_t r300_rb3d_color_mask;
  uint64_t r300_rb3d_color_pitch[4];
  uint64_t r300_rb3d_color_offset[4];
  uint64_t r300_rb3d_zcache_ctlstat;
  uint64_t r300_rb3d_dstcache_ctlstat;

  uint64_t rbbm_gui_cntl;
  uint64_t rbbm_status;
  uint64_t rbbm_soft_reset;

  uint64_t emu_register_stub[1024];

  //PLL CLK REGISTERS
uint64_t m_spll_ref_fb_div;




// MC registers
  uint64_t r300_mc_init_gfx_lat_timer;
  uint64_t r300_mc_init_misc_lat_timer;

//SE registers
  uint64_t r300_se_vport_xscale;
  uint64_t r300_se_vport_xoffset;
  uint64_t r300_se_vport_yscale;
  uint64_t r300_se_vport_yoffset;
  uint64_t r300_se_vport_zscale;
  uint64_t r300_se_vport_zoffset;
  uint64_t r300_se_vte_cntl;

//VAP registers

uint64_t r300_vap_cntl;
uint64_t r300_vap_cntl_status;
uint64_t r300_vap_output_vtx_fmt_0;
uint64_t r300_vap_output_vtx_fmt_1;
uint64_t r300_vap_input_cntl_0;
uint64_t r300_vap_input_cntl_1;
uint64_t r300_vap_input_route_0_0;
uint64_t r300_vap_input_route_0_1;
uint64_t r300_vap_input_route_0_2;
uint64_t r300_vap_input_route_0_3;
uint64_t r300_vap_input_route_0_4;
uint64_t r300_vap_input_route_0_5;
uint64_t r300_vap_input_route_0_6;
uint64_t r300_vap_input_route_0_7;
uint64_t r300_vap_input_route_1_0;
uint64_t r300_vap_input_route_1_1;
uint64_t r300_vap_input_route_1_2;
uint64_t r300_vap_input_route_1_3;
uint64_t r300_vap_input_route_1_4;
uint64_t r300_vap_input_route_1_5;
uint64_t r300_vap_input_route_1_6;
uint64_t r300_vap_input_route_1_7;
uint64_t r300_vap_pvs_upload_address;
uint64_t r300_vap_pvs_upload_data;





}RADVGARegs;
typedef struct RADVGAState {
    PCIDevice dev;
    VGACommonState vga;
    char *model;
    uint16_t dev_id;
    uint8_t mode;
    bool cursor_guest_mode;
    uint16_t cursor_size;
    uint32_t cursor_offset;
    QEMUCursor *cursor;
    QEMUTimer vblank_timer;
    bitbang_i2c_interface bbi2c;
    MemoryRegion io;
    MemoryRegion mm;
    MemoryRegion gart;
    MemoryRegion vram;
    AddressSpace gart_as;
    RADVGARegs regs;
} RADVGAState;

const char *r300_reg_name(int num);

void r300_2d_blt(RADVGAState *s);




#endif
