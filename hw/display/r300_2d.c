/*
 * QEMU ATI SVGA emulation
 * 2D engine functions
 *
 * Copyright (c) 2019 BALATON Zoltan
 *
 * This work is licensed under the GNU GPL license version 2 or later.
 */

#include "qemu/osdep.h"
#include "r300.h"
#include "r300_reg.h"
#include "radeon_reg.h"
#include "qemu/log.h"
#include "ui/pixel_ops.h"

/*
 * NOTE:
 * This is 2D _acceleration_ and supposed to be fast. Therefore, don't try to
 * reinvent the wheel (unlikely to get better with a naive implementation than
 * existing libraries) and avoid (poorly) reimplementing gfx primitives.
 * That is unnecessary and would become a performance problem. Instead, try to
 * map to and reuse existing optimised facilities (e.g. pixman) wherever
 * possible.
 */

static int r300_bpp_from_datatype(RADVGAState *s)
{
    switch (s->regs.dp_datatype & 0xf) {
    case 2:
        return 8;
    case 3:
    case 4:
        return 16;
    case 5:
        return 24;
    case 6:
        return 32;
    default:
        qemu_log_mask(LOG_UNIMP, "Unknown dst datatype %ld\n",
                      s->regs.dp_datatype & 0xf);
        return 0;
    }
}

#define DEFAULT_CNTL (s->regs.dp_gui_master_cntl & RADEON_GMC_DST_PITCH_OFFSET_CNTL)

void r300_2d_blt(RADVGAState *s)
{
    /* FIXME it is probably more complex than this and may need to be */
    /* rewritten but for now as a start just to get some output: */
    DisplaySurface *ds = qemu_console_surface(s->vga.con);
    qemu_log("%p %u ds: %p %d %d rop: %lx\n", s->vga.vram_ptr,
            s->vga.vbe_start_addr, surface_data(ds), surface_stride(ds),
            surface_bits_per_pixel(ds),
            (s->regs.dp_mix & RADEON_GMC_ROP3_MASK) >> 16);
    int dst_x = (s->regs.dp_cntl & RADEON_DST_X_LEFT_TO_RIGHT ?
                 s->regs.dst_x : s->regs.dst_x + 1 - s->regs.dst_width);
    int dst_y = (s->regs.dp_cntl & RADEON_DST_Y_TOP_TO_BOTTOM ?
                 s->regs.dst_y : s->regs.dst_y + 1 - s->regs.dst_height);
    int bpp = r300_bpp_from_datatype(s);
    int dst_stride = DEFAULT_CNTL ? s->regs.dst_pitch : s->regs.default_pitch;
    uint8_t *dst_bits = s->vga.vram_ptr + (DEFAULT_CNTL ?
                        s->regs.dst_offset : s->regs.default_offset);

    if (s->dev_id == PCI_DEVICE_ID_ATI_RADEON_9500_PRO) {
        dst_bits += s->regs.crtc_offset & 0x07ffffff;
        dst_stride *= bpp;
    }
    uint8_t *end = s->vga.vram_ptr + s->vga.vram_size;
    if (dst_bits >= end || dst_bits + dst_x + (dst_y + s->regs.dst_height) *
        dst_stride >= end) {
        qemu_log_mask(LOG_UNIMP, "blt outside vram not implemented\n");
        return;
    }
    qemu_log("%ld %ld %ld, %ld %ld %ld, (%ld,%ld) -> (%ld,%ld) %ldx%ld %c %c\n",
            s->regs.src_offset, s->regs.dst_offset, s->regs.default_offset,
            s->regs.src_pitch, s->regs.dst_pitch, s->regs.default_pitch,
            s->regs.src_x, s->regs.src_y, s->regs.dst_x, s->regs.dst_y,
            s->regs.dst_width, s->regs.dst_height,
            (s->regs.dp_cntl & RADEON_DST_X_LEFT_TO_RIGHT ? '>' : '<'),
            (s->regs.dp_cntl & RADEON_DST_Y_TOP_TO_BOTTOM ? 'v' : '^'));
    switch (s->regs.dp_mix & RADEON_GMC_ROP3_MASK) {
    case RADEON_ROP3_S:
    {
        int src_x = (s->regs.dp_cntl & RADEON_DST_X_LEFT_TO_RIGHT ?
                     s->regs.src_x : s->regs.src_x + 1 - s->regs.dst_width);
        int src_y = (s->regs.dp_cntl & RADEON_DST_Y_TOP_TO_BOTTOM ?
                     s->regs.src_y : s->regs.src_y + 1 - s->regs.dst_height);
        int src_stride = DEFAULT_CNTL ?
                         s->regs.src_pitch : s->regs.default_pitch;
        uint8_t *src_bits = s->vga.vram_ptr + (DEFAULT_CNTL ?
                            s->regs.src_offset : s->regs.default_offset);

        if (s->dev_id == PCI_DEVICE_ID_ATI_RADEON_9500_PRO) {
            src_bits += s->regs.crtc_offset & 0x07ffffff;
            src_stride *= bpp;
        }
        if (src_bits >= end || src_bits + src_x +
            (src_y + s->regs.dst_height) * src_stride >= end) {
            qemu_log_mask(LOG_UNIMP, "blt outside vram not implemented\n");
            return;
        }

        src_stride /= sizeof(uint64_t);
        dst_stride /= sizeof(uint64_t);
        qemu_log("pixman_blt(%p, %p, %d, %d, %d, %d, %d, %d, %d, %d, %ld, %ld)\n",
                src_bits, dst_bits, src_stride, dst_stride, bpp, bpp,
                src_x, src_y, dst_x, dst_y,
                s->regs.dst_width, s->regs.dst_height);
        if (s->regs.dp_cntl & RADEON_DST_X_LEFT_TO_RIGHT &&
            s->regs.dp_cntl & RADEON_DST_Y_TOP_TO_BOTTOM) {
            pixman_blt((uint32_t *)src_bits, (uint32_t *)dst_bits,
                       src_stride, dst_stride, bpp, bpp,
                       src_x, src_y, dst_x, dst_y,
                       s->regs.dst_width, s->regs.dst_height);
        } else {
            /* FIXME: We only really need a temporary if src and dst overlap */
            int llb = s->regs.dst_width * (bpp / 8);
            int tmp_stride = DIV_ROUND_UP(llb, sizeof(uint64_t));
            uint32_t *tmp = g_malloc(tmp_stride * sizeof(uint64_t) *
                                     s->regs.dst_height);
            pixman_blt((uint32_t *)src_bits, tmp,
                       src_stride, tmp_stride, bpp, bpp,
                       src_x, src_y, 0, 0,
                       s->regs.dst_width, s->regs.dst_height);
            pixman_blt(tmp, (uint32_t *)dst_bits,
                       tmp_stride, dst_stride, bpp, bpp,
                       0, 0, dst_x, dst_y,
                       s->regs.dst_width, s->regs.dst_height);
            g_free(tmp);
        }
        if (dst_bits >= s->vga.vram_ptr + s->vga.vbe_start_addr &&
            dst_bits < s->vga.vram_ptr + s->vga.vbe_start_addr +
            s->vga.vbe_regs[VBE_DISPI_INDEX_YRES] * s->vga.vbe_line_offset) {
            memory_region_set_dirty(&s->vga.vram, s->vga.vbe_start_addr +
                                    s->regs.dst_offset +
                                    dst_y * surface_stride(ds),
                                    s->regs.dst_height * surface_stride(ds));
        }
        s->regs.dst_x += s->regs.dst_width;
        s->regs.dst_y += s->regs.dst_height;
        break;
    }
    case RADEON_ROP3_P:
    case RADEON_ROP3_ZERO:
    case RADEON_ROP3_ONE:
    {
        uint64_t filler = 0;

        switch (s->regs.dp_mix & RADEON_GMC_ROP3_MASK) {
        case RADEON_ROP3_P:
            filler = s->regs.dp_brush_frgd_clr;
            break;
        case RADEON_ROP3_ZERO:
            filler = 0xffUL << 24 | rgb_to_pixel32(s->vga.palette[0],
                     s->vga.palette[1], s->vga.palette[2]);
            break;
        case RADEON_ROP3_ONE:
            filler = 0xffUL << 24 | rgb_to_pixel32(s->vga.palette[3],
                     s->vga.palette[4], s->vga.palette[5]);
            break;
        }

        dst_stride /= sizeof(uint64_t);
        qemu_log("pixman_fill(%p, %d, %d, %ld, %ld, %ld, %ld, %lx)\n",
                dst_bits, dst_stride, bpp,
                s->regs.dst_x, s->regs.dst_y,
                s->regs.dst_width, s->regs.dst_height,
                filler);
        pixman_fill((uint32_t *)dst_bits, dst_stride, bpp,
                    s->regs.dst_x, s->regs.dst_y,
                    s->regs.dst_width, s->regs.dst_height,
                    filler);
        if (dst_bits >= s->vga.vram_ptr + s->vga.vbe_start_addr &&
            dst_bits < s->vga.vram_ptr + s->vga.vbe_start_addr +
            s->vga.vbe_regs[VBE_DISPI_INDEX_YRES] * s->vga.vbe_line_offset) {
            memory_region_set_dirty(&s->vga.vram, s->vga.vbe_start_addr +
                                    s->regs.dst_offset +
                                    dst_y * surface_stride(ds),
                                    s->regs.dst_height * surface_stride(ds));
        }
        s->regs.dst_y += s->regs.dst_height;
        break;
    }
    default:
        qemu_log_mask(LOG_UNIMP, "Unimplemented r300_2d blt op %lx\n",
                      (s->regs.dp_mix & RADEON_GMC_ROP3_MASK) >> 16);
    }
}
