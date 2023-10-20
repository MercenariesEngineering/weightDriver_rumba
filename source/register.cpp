
#include "Maquina/Maquina.h"

using namespace maquina;

void register_weightDriver( Registry &r );

MAQUINA_DECLARE_PLUGINS
MAQUINA_REGISTER_PLUGINS
{
    register_weightDriver(r);
}
