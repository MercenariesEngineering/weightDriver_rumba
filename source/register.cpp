
#include "Maquina/Maquina.h"

using namespace rumba;

void register_weightDriver( Registry &r );

RUMBA_DECLARE_PLUGINS
RUMBA_REGISTER_PLUGINS
{
    register_weightDriver(r);
}
