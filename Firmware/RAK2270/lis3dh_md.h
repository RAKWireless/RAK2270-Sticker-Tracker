#ifndef __LIS3DH_MD_H__
#define __LIS3DH_MD_H__


#define LIS3D_INT1_PIN    PA10
#define LIS3D_INT2_PIN    PB14

void lis3dh_md_pause();
void lis3dh_md_resume();
void lis3dh_md_begin();
void lis3dh_md_end();
void lis3dh_init();
bool lis3dh_factory_test();
#endif
