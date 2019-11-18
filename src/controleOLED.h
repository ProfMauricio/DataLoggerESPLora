#ifndef __CONTROLEOLED_H__
#define __CONTROLEOLED_H__

#include <SSD1306.h>

typedef enum LinhasOLED {  USER_INFO,   BUFFER_INFO, LORA_INFO, STATUS_INFO, ULTIMA_LINHA}  TLinhasOLED;

class controleOLED
{
private:
    /* data */
    String espelhoOLED[ULTIMA_LINHA];
    SSD1306 *pDisplay;

public:
    controleOLED( SSD1306 *ptr) ;
    ~controleOLED();
    void atualizarVisualizacao();
    void modificarLinha( TLinhasOLED linha , String novaMensagem );
};

controleOLED::controleOLED(SSD1306 *ptr)
{
    int i = 0 ;
    for ( i = 0 ; i < ULTIMA_LINHA ; i++ )
        espelhoOLED[i] = "Linha " + String(i+1);
    
    this->pDisplay = ptr;
}

controleOLED::~controleOLED()
{
}

void controleOLED:: atualizarVisualizacao()
{
    int espacamento = 18;
    pDisplay->clear();
    for (int i = 0 ; i < ULTIMA_LINHA ; i++)
    {
        pDisplay->drawString(0 ,i*espacamento,  espelhoOLED[i]);
    }
    pDisplay->display();
}

void controleOLED::modificarLinha( TLinhasOLED linha , String novaMensagem )
{
    espelhoOLED[linha] = novaMensagem;
}


#endif