#ifndef _CST_LCD_H_
#define _CST_LCD_H_

extern const uint8_t Bell[8];
extern const uint8_t Horn[8];
extern const uint8_t SoftkeyInactive[8];
extern const uint8_t SoftkeyActive[8];

void setupDiagChars(void);
void setupSoftkeyChars(void);

// Splash Screen Characters
extern const uint8_t Splash1[8];
extern const uint8_t Splash2[8];
extern const uint8_t Splash3[8];
extern const uint8_t Splash4[8];
extern const uint8_t Splash5A[8];
extern const uint8_t Splash5C[8];
extern const uint8_t Splash6A[8];
extern const uint8_t Splash6B[8];
extern const uint8_t Splash6C[8];
extern const uint8_t Splash7A[8];
extern const uint8_t Splash7B[8];
extern const uint8_t Splash7C[8];
extern const uint8_t Splash8A[8];
extern const uint8_t Splash8B[8];

void displaySplashScreen(void);
void initLCD(void);

#endif

