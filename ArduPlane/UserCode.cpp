#include "Plane.h"

void Plane::user_init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "void Plane::user_init()");
    AP::fd_uartmav().init();
}

void Plane::user_100Hz() {
   gcs().send_text(MAV_SEVERITY_INFO, "void Plane::user_100Hz()");
    AP::fd_uartmav().update();
}
