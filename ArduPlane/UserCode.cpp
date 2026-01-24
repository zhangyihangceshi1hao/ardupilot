#include "Plane.h"

void Plane::user_init()
{
  
    AP::fd_uartmav().init();
}

void Plane::user_100Hz() {
 
    AP::fd_uartmav().update();
}
