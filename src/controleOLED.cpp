#include "controleOLED.h"



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
