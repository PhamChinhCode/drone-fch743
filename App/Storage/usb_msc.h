/**
 * @file    usb_msc.h
 * @brief   Doc the SD cua bo dieu khien qua cong USB, kieu o dia di dong.
 *
 * VI SAO PHAI CHON CHE DO LUC KHOI DONG
 *
 *   Cong USB cua bo mach chi co mot. Luc bay no dang lam duong telemetry
 *   kieu CDC (xem App/Telemetry/tlm_port.c). Muon may tinh nhin thay the SD
 *   thi phai enumerate lai thanh lop MSC - hai lop khong the cung song tren
 *   mot thiet bi neu khong dung mo ta ghep (composite), ma CubeMX khong sinh
 *   ra kieu do.
 *
 *   Nen cach chon la: GIU NUT K1 luc cam dien / bam reset thi vao che do the
 *   nho, tha ra binh thuong thi van la CDC nhu cu. Khong phai nap lai firmware
 *   de doi qua doi lai.
 *
 * VI SAO PHAI KHOA HOAN TOAN VIEC BAY TRONG CHE DO NAY
 *
 *   Khi may tinh gan the qua MSC, no toan quyen ghi vao tung sector, ke ca
 *   bang FAT. Neu firmware CUNG LUC do mount FatFs va ghi blackbox thi hai ben
 *   ghi de len nhau va he thong tep hong ngay.
 *
 *   Vi vay che do MSC:
 *     - KHONG goi blackbox_init(), tuc firmware khong he mount the;
 *     - CHAN ARM tuyet doi.
 *
 *   Muon bay lai thi rut USB, bam reset. Khong giu K1 nua la ve che do bay.
 */
#ifndef USB_MSC_H
#define USB_MSC_H

#include "fc_types.h"

/**
 * Doc nut K1 de biet nguoi dung co muon vao che do the nho khong.
 *
 * Goi MOT LAN o dau main(), sau MX_GPIO_Init() va truoc MX_USB_DEVICE_Init().
 * Ket qua duoc nho lai, cac lan goi sau tra ve dung gia tri do - de moi noi
 * trong chuong trinh deu thay cung mot cau tra loi.
 */
bool usb_msc_boot_requested(void);

/** Dang o che do the nho hay khong. An toan de goi bat cu luc nao. */
bool usb_msc_active(void);

#endif /* USB_MSC_H */
