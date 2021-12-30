#include <godot/gdnative_interface.h>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "debug_event_recorder.h"

using namespace godot;

static void register_types() {
	ClassDB::register_class<DebugEventRecorder>();
}

static void unregister_types()
{
}

extern "C" {

	GDNativeBool GDN_EXPORT mobility_library_init(const GDNativeInterface* p_interface, const GDNativeExtensionClassLibraryPtr p_library, GDNativeInitialization* r_initialization)
	{
		godot::GDExtensionBinding::InitObject init_obj(p_interface, p_library, r_initialization);

		init_obj.register_scene_initializer(register_types);
		init_obj.register_scene_terminator(unregister_types);

		return init_obj.init();
	}

}
