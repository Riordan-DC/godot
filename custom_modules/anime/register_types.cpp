#include "register_types.h"

#include "core/object/class_db.h"

#include "earcut.h"
//#include "gdblast/GodotBlast.h"
//#include "gdblast/GDBlastSingleton.h"
//#include "copy_skeleton_modifier_3d.h"
#include "anime.h"

#include "ozzgd.h"


//static NvBlastFramework* framework;

void initialize_anime_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

    // Register Godot C++ classes here
    ClassDB::register_class<Earcut>();
	//ClassDB::register_class<CopySkeletonModifier3D>();

    ClassDB::register_class<Anime>();
    ClassDB::register_class<TrackCache>();
    ClassDB::register_class<AnimationState>();
    ClassDB::register_class<BlendSpace2D>();

	ClassDB::register_class<OzzGD>();
	ClassDB::register_class<OzzAnimationState>();
	//ClassDB::register_class<OzzAnimationLibrary>();

    // GDREGISTER_CLASS(BlastActor);
	// GDREGISTER_CLASS(BlastGroup);
	// GDREGISTER_CLASS(BlastChunk);
	// GDREGISTER_CLASS(NvBlastFramework);
	
	// framework = memnew(NvBlastFramework); // Creates the singleton globally
    // Engine::get_singleton()->register_singleton("BlastFramework", NvBlastFramework::get_singleton());
}

void uninitialize_anime_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	
	// Engine::get_singleton()->unregister_singleton("BlastFramework");
	// memdelete(framework);
}

/*
extern "C" {
	// Initialization.
	GDExtensionBool GDE_EXPORT fps3_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
		godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
		
		init_obj.register_initializer(initialize_example_module);
		init_obj.register_terminator(uninitialize_example_module);
		init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
		
		return init_obj.init();
	}
}
*/