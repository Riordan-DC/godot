#include "register_types.h"

#include "core/object/class_db.h"

#include "ozzgd.h"
#include "anime.h"


void initialize_anime_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

    // Register Godot C++ classes here
    ClassDB::register_class<Anime>();
    ClassDB::register_class<TrackCache>();
    ClassDB::register_class<AnimationState>();
    ClassDB::register_class<BlendSpace2D>();


	ClassDB::register_class<OzzGD>();
	ClassDB::register_class<OzzAnimationState>();

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