/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __comm_H
#define __comm_H
#ifdef __cplusplus
 extern "C" {
#endif

typedef uint8_t bool;
#define FALSE   0
#define TRUE    1

void Comm_Init(void);
void Comm_Process(void);
void Clear_Rcv_Buf(void);
void Comm_Response(void);

#ifdef __cplusplus
}
#endif
#endif /*__ comm_H */