
#ifndef __EXTUNIT_H__
#define __EXTUNIT_H__

int xu_get_len(int fd, unsigned int uintID, unsigned long selector, unsigned long* pSize);
int xu_set_cur(int fd, unsigned int uintID, unsigned long selector, unsigned long size, unsigned char* pValue);
int xu_get_cur(int fd, unsigned int uintID, unsigned long selector, unsigned long size, unsigned char* pValue);
int xu_get_min(int fd, unsigned int uintID, unsigned long selector, unsigned long size, unsigned char* pValue);
int xu_get_max(int fd, unsigned int uintID, unsigned long selector, unsigned long size, unsigned char* pValue);
int xu_get_def(int fd, unsigned int uintID, unsigned long selector, unsigned long size, unsigned char* pValue);

#endif /* __EXTUNIT_H__ */