#ifndef __AST_FB_IOCTL_H__
#define __AST_FB_IOCTL_H__

struct astfb_dfbinfo {
	unsigned long ulFBSize;
	unsigned long ulFBPhys;

	unsigned long ulCMDQSize;
	unsigned long ulCMDQOffset;

	unsigned long ul2DMode;
};

#define ASTFB_GET_DFBINFO	_IOR(0xF3,0x00,struct astfb_dfbinfo)

#endif /* !__AST_FB_IOCTL_H__ */
