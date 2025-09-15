# custom.py

module_basis_universal_enabled = "no"
module_bmp_enabled = "no"
module_camera_enabled = "no"
module_csg_enabled = "no"
module_dds_enabled = "no"
module_enet_enabled = "no"
module_gridmap_enabled = "no"
module_hdr_enabled = "no"
module_jsonrpc_enabled = "no"
module_ktx_enabled = "no"
module_mbedtls_enabled = "no"
module_meshoptimizer_enabled = "no"
module_minimp3_enabled = "no"
module_mobile_vr_enabled = "no"
module_msdfgen_enabled= "no"
module_multiplayer_enabled = "no"
module_noise_enabled = "no"
module_navigation_enabled = "no"
module_ogg_enabled = "no"
module_openxr_enabled = "no"
module_raycast_enabled = "no"
module_regex_enabled = "no"
module_squish_enabled = "no"
module_tga_enabled = "no"
module_theora_enabled = "no"
module_tinyexr_enabled = "no"
module_upnp_enabled = "no"
module_vhacd_enabled = "no"
module_vorbis_enabled = "no"
module_webrtc_enabled = "no"
module_websocket_enabled = "no"
module_webxr_enabled = "no"
module_zip_enabled = "no"

debug_symbols="no"
optimize="size"
lto="full" # Slower build times, smaller export size

disable_3d="yes"
disable_advanced_gui="no"

deprecated="no"  # Disables deprecated features
vulkan="no"      # Disables the Vulkan driver (used in Forward+/Mobile Renderers)
use_volk="no"    # Disables more Vulkan stuff
openxr="no"      # Disables Virtual Reality/Augmented Reality stuff
minizip="no"     # Disables ZIP archive support
graphite="no"    # Disables SIL Graphite smart fonts support

#modules_enabled_by_default="no"
module_gdscript_enabled="yes"
module_text_server_fb_enabled="yes"
module_freetype_enabled="yes" # Needed alongside a text server for text to render correctly
module_svg_enabled="yes"
module_webp_enabled="yes"
module_godot_physics_2d_enabled="yes"