#ifndef _DEBUG_UTILS_H
#define _DEBUG_UTILS_H

inline void ITM_SendCharOnChannel (uint32_t ch, uint32_t port);
inline void ITM_SendStrOnChannel (uint8_t* str, uint32_t port);

#endif
