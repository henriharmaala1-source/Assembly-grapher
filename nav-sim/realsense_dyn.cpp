#include "realsense_dyn.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace rsdyn {
namespace {

// --- the C ABI we use -------------------------------------------------------
// Opaque pointers, exactly as librealsense treats them.
using rs2_error_pp = void**;

struct Fns {
    int         (*get_api_version)(rs2_error_pp);
    const char* (*get_error_message)(void*);
    void        (*free_error)(void*);

    void* (*create_context)(int, rs2_error_pp);
    void  (*delete_context)(void*);
    void* (*create_pipeline)(void*, rs2_error_pp);
    void  (*delete_pipeline)(void*);
    void* (*create_config)(rs2_error_pp);
    void  (*delete_config)(void*);
    void  (*config_enable_stream)(void*, int stream, int index, int width,
                                  int height, int format, int framerate,
                                  rs2_error_pp);
    void* (*pipeline_start_with_config)(void*, void*, rs2_error_pp);
    void  (*pipeline_stop)(void*, rs2_error_pp);
    void  (*delete_pipeline_profile)(void*);

    void* (*pipeline_wait_for_frames)(void*, unsigned timeout_ms, rs2_error_pp);
    int   (*embedded_frames_count)(void*, rs2_error_pp);
    void* (*extract_frame)(void*, int index, rs2_error_pp);
    const void* (*get_frame_data)(const void*, rs2_error_pp);
    int   (*get_frame_width)(const void*, rs2_error_pp);
    int   (*get_frame_height)(const void*, rs2_error_pp);
    void  (*release_frame)(void*);
    const void* (*get_frame_stream_profile)(const void*, rs2_error_pp);
    void  (*get_video_stream_intrinsics)(const void*, Intrinsics*, rs2_error_pp);
    // rs2_get_stream_profile_data(mode, stream, format, index, unique_id,
    // framerate, error). SIX out-params, not five. The previous declaration
    // here was one short, which is a stack-corrupting call the moment anything
    // uses it -- it never had, so it sat as a landmine rather than a bug.
    void  (*get_stream_profile_data)(const void*, int*, int*, int*, int*, int*,
                                     rs2_error_pp);
    double (*get_frame_timestamp)(const void*, rs2_error_pp);
    void* (*get_frame_sensor)(const void*, rs2_error_pp);

    void* (*pipeline_profile_get_device)(void*, rs2_error_pp);
    void  (*delete_device)(void*);
    const char* (*get_device_info)(const void*, int, rs2_error_pp);
    void* (*query_sensors)(const void*, rs2_error_pp);
    int   (*get_sensors_count)(const void*, rs2_error_pp);
    void* (*create_sensor)(const void*, int, rs2_error_pp);
    void  (*delete_sensor)(void*);
    void  (*delete_sensor_list)(void*);
    float (*get_depth_scale)(void*, rs2_error_pp);
    int   (*supports_option)(const void*, int, rs2_error_pp);
    void  (*set_option)(const void*, int, float, rs2_error_pp);
    int   (*is_sensor_extendable_to)(const void*, int, rs2_error_pp);
};

Fns g;
void* g_handle = nullptr;
std::string g_path;
int  g_api = 0;

void* sym(const char* name) {
#ifdef _WIN32
    return (void*)GetProcAddress((HMODULE)g_handle, name);
#else
    return dlsym(g_handle, name);
#endif
}

void* openLib(const char* name) {
#ifdef _WIN32
    return (void*)LoadLibraryA(name);
#else
    return dlopen(name, RTLD_NOW | RTLD_LOCAL);
#endif
}

// Every place the library plausibly is.
//
// The fixed list below was not enough on a real machine: the SDK was installed
// and none of the four hard-coded paths existed, because the product folder is
// named by an installer whose vendor changed. So the fixed list is only the
// first pass -- after it, every directory under Program Files whose name
// mentions RealSense is enumerated and the usual subfolders tried inside it.
//
// std::filesystem rather than FindFirstFile, so the search can be exercised on
// a machine that is not Windows. A Windows-only code path that nothing else can
// run is how the last three of these bugs survived.
const char* kCandidates[] = {
#ifdef _WIN32
    "realsense2.dll",
    "C:/Program Files (x86)/Intel RealSense SDK 2.0/bin/x64/realsense2.dll",
    "C:/Program Files/Intel RealSense SDK 2.0/bin/x64/realsense2.dll",
    "C:/Program Files (x86)/RealSense SDK 2.0/bin/x64/realsense2.dll",
    "C:/Program Files/RealSense SDK 2.0/bin/x64/realsense2.dll",
#else
    "librealsense2.so",
    "librealsense2.so.2",
    "/usr/lib/x86_64-linux-gnu/librealsense2.so",
    "/usr/local/lib/librealsense2.so",
#endif
};

#ifdef _WIN32
const char* kLibName = "realsense2.dll";
#else
const char* kLibName = "librealsense2.so";
#endif

// Roots to hunt under, and the subfolders an install puts the library in.
std::vector<std::string> searchRoots() {
    std::vector<std::string> roots;
#ifdef _WIN32
    for (const char* env : {"ProgramW6432", "ProgramFiles", "ProgramFiles(x86)",
                            "LOCALAPPDATA"}) {
        const char* v = std::getenv(env);
        if (v && *v) roots.push_back(v);
    }
    roots.push_back("C:/Program Files");
    roots.push_back("C:/Program Files (x86)");
#else
    roots.push_back("/usr/lib");
    roots.push_back("/usr/local/lib");
    roots.push_back("/opt");
#endif
    if (const char* v = std::getenv("REALSENSE2_DIR")) if (*v) roots.push_back(v);
    return roots;
}

// Any directory whose name mentions realsense, one level under each root, plus
// the roots themselves. Returns full candidate paths to the library.
std::vector<std::string> globCandidates() {
    namespace fs = std::filesystem;
    std::vector<std::string> out;
    const char* subs[] = {"bin/x64", "bin", "lib/x64", "lib", ""};
    auto tryDir = [&](const fs::path& d) {
        for (const char* sub : subs) {
            fs::path p = *sub ? (d / sub / kLibName) : (d / kLibName);
            std::error_code ec;
            if (fs::exists(p, ec)) out.push_back(p.string());
        }
    };
    for (const std::string& root : searchRoots()) {
        std::error_code ec;
        fs::path r(root);
        if (!fs::exists(r, ec)) continue;
        tryDir(r);                                   // the root itself
        for (fs::directory_iterator it(r, fs::directory_options::skip_permission_denied, ec), end;
             it != end && !ec; it.increment(ec)) {
            std::error_code ec2;
            if (!it->is_directory(ec2)) continue;
            std::string name = it->path().filename().string();
            for (char& c : name) c = char(std::tolower((unsigned char)c));
            if (name.find("realsense") != std::string::npos) tryDir(it->path());
        }
    }
    return out;
}

// An error out-parameter that cleans itself up.
struct Err {
    void* e = nullptr;
    ~Err() { if (e && g.free_error) g.free_error(e); }
    rs2_error_pp operator&() { return &e; }
    bool bad() const { return e != nullptr; }
    std::string msg() const {
        return (e && g.get_error_message) ? g.get_error_message(e) : "unknown error";
    }
};

bool resolveAll(std::string* missing) {
    struct { void** slot; const char* name; } table[] = {
        {(void**)&g.get_api_version,   "rs2_get_api_version"},
        {(void**)&g.get_error_message, "rs2_get_error_message"},
        {(void**)&g.free_error,        "rs2_free_error"},
        {(void**)&g.create_context,    "rs2_create_context"},
        {(void**)&g.delete_context,    "rs2_delete_context"},
        {(void**)&g.create_pipeline,   "rs2_create_pipeline"},
        {(void**)&g.delete_pipeline,   "rs2_delete_pipeline"},
        {(void**)&g.create_config,     "rs2_create_config"},
        {(void**)&g.delete_config,     "rs2_delete_config"},
        {(void**)&g.config_enable_stream, "rs2_config_enable_stream"},
        {(void**)&g.pipeline_start_with_config, "rs2_pipeline_start_with_config"},
        {(void**)&g.pipeline_stop,     "rs2_pipeline_stop"},
        {(void**)&g.delete_pipeline_profile, "rs2_delete_pipeline_profile"},
        {(void**)&g.pipeline_wait_for_frames, "rs2_pipeline_wait_for_frames"},
        {(void**)&g.embedded_frames_count, "rs2_embedded_frames_count"},
        {(void**)&g.extract_frame,     "rs2_extract_frame"},
        {(void**)&g.get_frame_data,    "rs2_get_frame_data"},
        {(void**)&g.get_frame_width,   "rs2_get_frame_width"},
        {(void**)&g.get_frame_height,  "rs2_get_frame_height"},
        {(void**)&g.release_frame,     "rs2_release_frame"},
        {(void**)&g.get_frame_stream_profile, "rs2_get_frame_stream_profile"},
        {(void**)&g.get_video_stream_intrinsics, "rs2_get_video_stream_intrinsics"},
        {(void**)&g.get_stream_profile_data, "rs2_get_stream_profile_data"},
        {(void**)&g.get_frame_timestamp, "rs2_get_frame_timestamp"},
        {(void**)&g.get_frame_sensor,  "rs2_get_frame_sensor"},
        {(void**)&g.pipeline_profile_get_device, "rs2_pipeline_profile_get_device"},
        {(void**)&g.delete_device,     "rs2_delete_device"},
        {(void**)&g.get_device_info,   "rs2_get_device_info"},
        {(void**)&g.query_sensors,     "rs2_query_sensors"},
        {(void**)&g.get_sensors_count, "rs2_get_sensors_count"},
        {(void**)&g.create_sensor,     "rs2_create_sensor"},
        {(void**)&g.delete_sensor,     "rs2_delete_sensor"},
        {(void**)&g.delete_sensor_list,"rs2_delete_sensor_list"},
        {(void**)&g.get_depth_scale,   "rs2_get_depth_scale"},
        {(void**)&g.supports_option,   "rs2_supports_option"},
        {(void**)&g.set_option,        "rs2_set_option"},
        {(void**)&g.is_sensor_extendable_to, "rs2_is_sensor_extendable_to"},
    };
    std::string bad;
    for (const auto& t : table) {
        *t.slot = sym(t.name);
        if (!*t.slot) { if (!bad.empty()) bad += ", "; bad += t.name; }
    }
    if (!bad.empty()) { if (missing) *missing = bad; return false; }
    return true;
}

}  // namespace

bool loaded() { return g_handle != nullptr; }
const std::string& libraryPath() { return g_path; }
int apiVersion() { return g_api; }

bool load(std::string* err) {
    if (g_handle) return true;
    std::string tried;

    // Fixed list first (cheap, and the bare name lets the OS loader do its own
    // search), then anything the filesystem hunt turns up.
    std::vector<std::string> all;
    for (const char* c : kCandidates) all.push_back(c);
    for (const std::string& c : globCandidates()) all.push_back(c);

    for (const std::string& cand : all) {
        void* h = openLib(cand.c_str());
        if (!h) {
            if (!tried.empty()) tried += "\n";
            tried += "  " + cand;
            continue;
        }
        g_handle = h;
        std::string missing;
        if (!resolveAll(&missing)) {
            g_handle = nullptr;
            if (err)
                *err = std::string("loaded ") + cand +
                       " but it is missing symbols: " + missing +
                       "\n  (is it really librealsense?)";
            return false;
        }
        g_path = cand;
        // Ask the LIBRARY what version it is and hand that straight back to it.
        // Compiling a version number in is how a working install gets rejected
        // for being a different point release than the build machine had.
        Err e;
        g_api = g.get_api_version(&e);
        return true;
    }
    if (err)
        *err = "could not load librealsense. Tried:\n" + tried +
               "\n  If the SDK or the Viewer is installed, the library exists -- "
               "put its folder on PATH, or copy realsense2.dll beside the exe.";
    return false;
}

// ---------------------------------------------------------------------------
bool Pipeline::start(int width, int height, int fps,
                     bool wantIR, bool wantIMU) {
    if (!load(&err_)) return false;
    stop();

    Err e;
    ctx_ = g.create_context(g_api, &e);
    if (e.bad() || !ctx_) { err_ = "create_context: " + e.msg(); return false; }

    Err e2;
    pipe_ = g.create_pipeline(ctx_, &e2);
    if (e2.bad() || !pipe_) { err_ = "create_pipeline: " + e2.msg(); stop(); return false; }

    Err e3;
    void* cfg = g.create_config(&e3);
    if (e3.bad() || !cfg) { err_ = "create_config: " + e3.msg(); stop(); return false; }

    Err e4;
    g.config_enable_stream(cfg, STREAM_DEPTH, 0, width, height, FORMAT_Z16, fps, &e4);
    if (e4.bad()) { err_ = "enable_stream: " + e4.msg(); g.delete_config(cfg); stop(); return false; }

    // OPTIONAL STREAMS, EACH IN ITS OWN ATTEMPT. A D435 without the i has no
    // motion sensor, and a USB 2 link will refuse the extra bandwidth -- in
    // both cases the correct outcome is depth without the extra, not a dead
    // camera. So a failure here is recorded and swallowed.
    haveIR_ = false; haveIMU_ = false;
    if (wantIR) {
        Err ei;
        // Index 1 is the LEFT imager. Depth is computed in its frame, so it is
        // the one already registered with depth; index 2 would need alignment.
        g.config_enable_stream(cfg, STREAM_INFRARED, 1, width, height,
                               FORMAT_Y8, fps, &ei);
        haveIR_ = !ei.bad();
    }
    if (wantIMU) {
        Err eg, ea;
        // Motion streams carry no resolution. Rate 0 asks for the device
        // default, which avoids guessing at a value the firmware may not offer.
        g.config_enable_stream(cfg, STREAM_GYRO,  0, 0, 0, FORMAT_MOTION_XYZ32F, 0, &eg);
        g.config_enable_stream(cfg, STREAM_ACCEL, 0, 0, 0, FORMAT_MOTION_XYZ32F, 0, &ea);
        haveIMU_ = !eg.bad() && !ea.bad();
    }

    Err e5;
    profile_ = g.pipeline_start_with_config(pipe_, cfg, &e5);
    g.delete_config(cfg);
    if (e5.bad() || !profile_) {
        err_ = "start: " + e5.msg();
        stop();
        return false;
    }

    // Depth scale and the emitter both live on the DEPTH SENSOR, which has to
    // be picked out of the device's sensor list -- there is no first_depth_sensor
    // in the C API, that is a convenience of the C++ and Python wrappers.
    Err e6;
    void* dev = g.pipeline_profile_get_device(profile_, &e6);
    if (!e6.bad() && dev) {
        Err e7;
        void* list = g.query_sensors(dev, &e7);
        if (!e7.bad() && list) {
            Err e8;
            const int n = g.get_sensors_count(list, &e8);
            for (int i = 0; i < n && !sensor_; ++i) {
                Err e9;
                void* s = g.create_sensor(list, i, &e9);
                if (e9.bad() || !s) continue;
                Err e10;
                // RS2_EXTENSION_DEPTH_SENSOR = 12
                if (g.is_sensor_extendable_to(s, 12, &e10) && !e10.bad()) {
                    sensor_ = s;
                } else {
                    g.delete_sensor(s);
                }
            }
            g.delete_sensor_list(list);
        }
        g.delete_device(dev);
    }
    if (sensor_) {
        Err e11;
        const float sc = g.get_depth_scale(sensor_, &e11);
        if (!e11.bad() && sc > 0.f) depthScale_ = sc;
    }
    err_.clear();
    return true;
}

void Pipeline::stop() {
    if (sensor_)  { g.delete_sensor(sensor_); sensor_ = nullptr; }
    if (profile_) { g.delete_pipeline_profile(profile_); profile_ = nullptr; }
    if (pipe_) {
        Err e;
        g.pipeline_stop(pipe_, &e);
        g.delete_pipeline(pipe_);
        pipe_ = nullptr;
    }
    if (ctx_) { g.delete_context(ctx_); ctx_ = nullptr; }
}

bool Pipeline::setEmitter(bool on) {
    if (!sensor_) return false;
    Err e;
    if (!g.supports_option(sensor_, OPTION_EMITTER_ENABLED, &e) || e.bad()) return false;
    Err e2;
    g.set_option(sensor_, OPTION_EMITTER_ENABLED, on ? 1.f : 0.f, &e2);
    return !e2.bad();
}

std::string Pipeline::deviceInfo(int info) const {
    if (!profile_) return "";
    Err e;
    void* dev = g.pipeline_profile_get_device(profile_, &e);
    if (e.bad() || !dev) return "";
    Err e2;
    const char* s = g.get_device_info(dev, info, &e2);
    std::string out = (!e2.bad() && s) ? s : "";
    g.delete_device(dev);
    return out;
}

bool Pipeline::waitDepth(std::vector<uint16_t>& out, int& w, int& h, int timeoutMs) {
    return waitFrames(out, w, h, nullptr, nullptr, timeoutMs);
}

bool Pipeline::waitFrames(std::vector<uint16_t>& out, int& w, int& h,
                          std::vector<uint8_t>* ir, std::vector<Motion>* motion,
                          int timeoutMs) {
    if (!pipe_) { err_ = "pipeline not started"; return false; }
    Err e;
    void* fs = g.pipeline_wait_for_frames(pipe_, unsigned(timeoutMs), &e);
    if (e.bad() || !fs) { err_ = "wait_for_frames: " + e.msg(); return false; }

    Err e2;
    const int n = g.embedded_frames_count(fs, &e2);
    bool got = false;
    for (int i = 0; i < n; ++i) {
        Err e3;
        void* f = g.extract_frame(fs, i, &e3);
        if (e3.bad() || !f) continue;

        // WHICH STREAM IS THIS? Asking the profile is the only reliable answer:
        // classifying by size would confuse a Y8 infrared frame with nothing in
        // particular, and motion frames have no width at all.
        int st = -1, fmt = 0, idx = 0, uid = 0, rate = 0;
        { Err ep;
          const void* sp = g.get_frame_stream_profile(f, &ep);
          if (!ep.bad() && sp) {
              Err ed;
              g.get_stream_profile_data(sp, &st, &fmt, &idx, &uid, &rate, &ed);
              if (ed.bad()) st = -1;
          } }

        if (motion && (st == STREAM_GYRO || st == STREAM_ACCEL)) {
            Err em;
            const void* d = g.get_frame_data(f, &em);
            if (!em.bad() && d) {
                const float* v = static_cast<const float*>(d);
                Err et;
                const double ts = g.get_frame_timestamp ? g.get_frame_timestamp(f, &et) : 0.0;
                motion->push_back({st == STREAM_GYRO, v[0], v[1], v[2],
                                   et.bad() ? 0.0 : ts});
            }
            g.release_frame(f);
            continue;
        }

        if (ir && st == STREAM_INFRARED) {
            Err ew, eh2, em;
            const int iw = g.get_frame_width(f, &ew), ih = g.get_frame_height(f, &eh2);
            const void* d = g.get_frame_data(f, &em);
            if (!ew.bad() && !eh2.bad() && !em.bad() && d && iw > 0 && ih > 0) {
                ir->resize(size_t(iw) * ih);
                std::memcpy(ir->data(), d, ir->size());
            }
            g.release_frame(f);
            continue;
        }

        if (got || (st != -1 && st != STREAM_DEPTH)) { g.release_frame(f); continue; }

        Err e4, e5;
        const int fw = g.get_frame_width(f, &e4), fh = g.get_frame_height(f, &e5);
        if (!e4.bad() && !e5.bad() && fw > 0 && fh > 0) {
            Err e6;
            const void* data = g.get_frame_data(f, &e6);
            if (!e6.bad() && data) {
                w = fw; h = fh;
                out.resize(size_t(fw) * fh);
                std::memcpy(out.data(), data, out.size() * sizeof(uint16_t));
                // Intrinsics come from THIS frame's profile, so they always
                // describe the stream actually being delivered rather than the
                // one that was requested -- which differ whenever the device
                // substitutes a mode, as a USB 2 link routinely does.
                Err e7;
                const void* sp = g.get_frame_stream_profile(f, &e7);
                if (!e7.bad() && sp) {
                    Err e8;
                    g.get_video_stream_intrinsics(sp, &intr_, &e8);
                }
                got = true;
            }
        }
        g.release_frame(f);
    }
    g.release_frame(fs);
    if (!got) err_ = "frameset contained no usable depth frame";
    return got;
}

}  // namespace rsdyn
