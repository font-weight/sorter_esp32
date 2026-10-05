// End-to-end SOFTWARE experiment. All geometry, colors, lighting and the
// simulated removal of parts are invented. No camera or actuator is connected.
#include <ColorDetector.h>
#include <Homography.h>
#include <MotionCore.h>
#include <Protocol.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

namespace {
using namespace sorter;
constexpr uint16_t WIDTH = 320, HEIGHT = 240;
constexpr uint32_t SESSION = 20260923, CAMERA_BOOT = 817;
const ColorRange COLORS[] = {
  {1,120,255,0,110,0,110,60},
  {2,0,120,100,255,0,130,45},
  {3,0,110,0,150,110,255,50}
};

struct SyntheticPart {
  uint8_t classId;
  uint16_t left, top;
  bool present;
};
struct SceneRecord {
  uint32_t sequence;
  uint8_t count;
  Detection target;
};
struct ExperimentReport {
  std::vector<SceneRecord> scans;
  std::vector<unsigned> releasedClasses;
  uint32_t virtualElapsedMs = 0;
  size_t commandedPoses = 0;
  uint32_t corruptParserErrors = 0;
  size_t corruptAdditionalPoses = 0;
  size_t wrongCalibrationAdditionalPoses = 0;
  uint32_t ignoredDuplicates = 0;
};

bool samePose(const JointPose& a, const JointPose& b) {
  for (unsigned j = 0; j < 4; ++j) if (a.us[j] != b.us[j]) return false;
  return true;
}

struct VirtualArm : MotionIO {
  const MotionCalibration& calibration;
  std::vector<SyntheticPart>& world;
  std::vector<std::string> requests;
  std::vector<JointPose> poses;
  std::vector<unsigned> releases;
  unsigned detaches = 0;
  int selectedPart = -1;
  JointPose expectedPick = {};
  bool descendedAtExpectedTarget = false;

  VirtualArm(const MotionCalibration& c, std::vector<SyntheticPart>& parts)
      : calibration(c), world(parts) {}
  bool enableAt(const JointPose& pose) override {
    poses.push_back(pose); return true;
  }
  bool writePose(const JointPose& pose) override {
    assert(poseWithinLimits(pose, calibration));
    poses.push_back(pose);
    if (selectedPart < 0) return true;
    if (samePose(pose, expectedPick)) descendedAtExpectedTarget = true;
    for (unsigned bin = 0; bin < 3; ++bin) {
      if (!samePose(pose, calibration.binDropOpen[bin].joints)) continue;
      assert(descendedAtExpectedTarget);
      assert(world[size_t(selectedPart)].present);
      assert(world[size_t(selectedPart)].classId == bin+1);
      // This is the explicit simulated-world assumption, not a measured grip.
      // Remove a part only after the controller reaches its correct bin and
      // commands the calibrated fully-open release pose.
      world[size_t(selectedPart)].present = false;
      releases.push_back(bin+1);
      selectedPart = -1;
      break;
    }
    return true;
  }
  void detachOutputs() override { ++detaches; }
  bool requestScene(uint32_t session, uint32_t seq, uint32_t id) override {
    Packet request;
    request.type = 'Q'; request.session = session; request.seq = seq;
    snprintf(request.payload, sizeof(request.payload), "%u", unsigned(id));
    char frame[kMaxFrame];
    if (!encodePacket(request, frame, sizeof(frame))) return false;
    requests.emplace_back(frame);
    return true;
  }
  void expectTarget(size_t index, const Detection& target) {
    assert(selectedPart < 0 && index < world.size());
    selectedPart = int(index);
    descendedAtExpectedTarget = false;
    CalibratedPose pickup;
    assert(interpolateGrid(calibration, target.x10, target.y10, false, pickup));
    expectedPick = pickup.joints;
  }
};

class Pipeline {
 public:
  // The two red objects are separated by more than the target-match radius;
  // otherwise a neighbour could conservatively be treated as a failed pickup.
  std::vector<SyntheticPart> world = {{1,80,60,true},{1,228,120,true},{3,140,180,true}};
  MotionCalibration calibration = exampleVirtualCalibration();
  MotionSettings settings = defaultMotionSettings();
  VirtualArm arm;
  MotionController controller;
  Homography homography;
  LineParser cameraParser, controllerParser;
  std::vector<uint8_t> pixels, labels;
  std::vector<uint32_t> queue;
  std::vector<SceneRecord> scans;
  size_t nextRequest = 0;
  uint32_t now = 0;

  Pipeline() : arm(calibration,world), controller(calibration,settings,arm),
               pixels(size_t(WIDTH)*HEIGHT*2), labels(size_t(WIDTH)*HEIGHT),
               queue(size_t(WIDTH)*HEIGHT) {
    assert(settings.virtualMode && !settings.outputsEnabled);
    assert(!calibration.geometryConfirmed && !calibration.routesVerified);
    assert(!calibrationValid(calibration,true)); // Never a hardware calibration.
    // Invented planar mapping: X=(u-160)/4 mm, Y=80+v/6 mm.
    const double matrix[] = {0.25,0,-40, 0,1.0/6,80, 0,0,1};
    const Point2 support[] = {{40,1},{280,1},{280,239},{40,239}};
    assert(homography.configure(matrix,support,4));
  }

  void tick() {
    now += 20;
    controller.tick(now,true,true); // Simulated hardware-enable and supply.
    assert(controller.state() != MotionState::Fault ||
           controller.error() == MotionError::CameraTimeout ||
           controller.error() == MotionError::CalibrationMismatch);
  }
  void start() {
    controller.confirmOperatorPark();
    assert(controller.arm(now,SESSION,true,true));
    untilRequest();
  }
  void untilRequest() {
    const uint32_t began = now;
    while (nextRequest >= arm.requests.size() && uint32_t(now-began) < 120000 &&
           controller.state() != MotionState::Fault && controller.state() != MotionState::Done) tick();
    assert(nextRequest < arm.requests.size());
    assert(controller.state() == MotionState::WaitScene);
  }

  void renderWorld() {
    std::fill(pixels.begin(),pixels.end(),uint8_t(0));
    for (const SyntheticPart& part : world) {
      if (!part.present) continue;
      const uint16_t color = part.classId == 1 ? 0xF800 : part.classId == 2 ? 0x07E0 : 0x001F;
      for (unsigned y = part.top; y < unsigned(part.top)+8; ++y) {
        for (unsigned x = part.left; x < unsigned(part.left)+8; ++x) {
          const size_t offset = (size_t(y)*WIDTH+x)*2;
          pixels[offset] = uint8_t(color >> 8);
          pixels[offset+1] = uint8_t(color);
        }
      }
    }
  }

  // A real request wire frame traverses the real byte-stream parser before
  // invoking the detector. There is no manually constructed detection scene.
  std::string captureReply(bool wrongCalibration = false) {
    assert(nextRequest < arm.requests.size());
    const std::string& wireRequest = arm.requests[nextRequest++];
    Packet request;
    unsigned decodedRequests = 0;
    for (char ch : wireRequest) if (cameraParser.feed(ch,request)) ++decodedRequests;
    assert(decodedRequests == 1 && request.type == 'Q');
    uint32_t id = 0;
    assert(parseU32(request.payload,id) && id == calibration.calibrationId);
    renderWorld();
    const DetectorConfig detector = {COLORS,3,{40,1,281,240},10,500,true};
    DetectorWorkspace workspace = {labels.data(),labels.size(),queue.data(),queue.size()};
    PixelBlob blobs[kMaxObjects];
    const DetectionResult found = detectRgb565BE(pixels.data(),pixels.size(),WIDTH,HEIGHT,
                                                detector,workspace,blobs,kMaxObjects);
    assert(found.status == DetectionStatus::Ok);
    Scene scene;
    scene.cameraBoot = CAMERA_BOOT;
    scene.calibrationId = id + (wrongCalibration ? 1 : 0);
    for (size_t i = 0; i < found.count; ++i) {
      Detection detection;
      detection.classId = blobs[i].classId;
      detection.pixels = blobs[i].pixels;
      assert(homography.mapTenths(blobs[i].centerX,blobs[i].centerY,detection.x10,detection.y10));
      scene.objects[scene.count++] = detection;
    }
    SceneRecord record = {request.seq,scene.count,{}};
    if (scene.count) record.target = scene.objects[0];
    scans.push_back(record);
    // Set the external world's expected transfer only for a valid response.
    // This does not choose the controller's motion; the actual commanded
    // pickup and release poses are checked independently by VirtualArm.
    if (scene.count && !wrongCalibration) {
      size_t index = 0;
      while (index < world.size() && !world[index].present) ++index;
      assert(index < world.size() && world[index].classId == scene.objects[0].classId);
      arm.expectTarget(index,scene.objects[0]);
    }
    Packet response;
    response.type = 'D'; response.session = request.session; response.seq = request.seq;
    assert(encodeScene(scene,response.payload,sizeof(response.payload)));
    char wireReply[kMaxFrame];
    assert(encodePacket(response,wireReply,sizeof(wireReply)));
    return std::string(wireReply);
  }

  unsigned deliver(const std::string& bytes) {
    Packet packet;
    unsigned decodedPackets = 0;
    // Tick precedes response processing, matching the firmware input/timeout
    // ordering. No virtual time passes between individual UART bytes here.
    controller.tick(now,true,true);
    for (char ch : bytes) {
      if (!controllerParser.feed(ch,packet)) continue;
      ++decodedPackets;
      assert(packet.type == 'D');
      Scene scene;
      assert(decodeScene(packet.payload,scene));
      controller.receiveScene(packet.session,packet.seq,scene,now);
    }
    return decodedPackets;
  }
};

void normalClosedLoop(ExperimentReport& report) {
  Pipeline rig;
  rig.start();
  for (unsigned remaining = 3; ; --remaining) {
    assert(rig.controller.state() == MotionState::WaitScene);
    const std::string reply = rig.captureReply();
    assert(rig.scans.back().count == remaining);
    assert(rig.deliver(reply) == 1);
    if (remaining == 3) {
      // The input image held two red parts; the scene retains two separate
      // coordinates. The expected first coordinate comes from pixel geometry.
      assert(rig.scans.front().target.classId == 1);
      assert(rig.scans.front().target.x10 == -191 && rig.scans.front().target.y10 == 906);
      assert(rig.scans.front().target.pixels == 64);
      assert(rig.deliver(reply) == 1); // UART-valid duplicate must not start twice.
      assert(rig.controller.attempts() == 1 && rig.controller.staleReplies() == 1);
    }
    if (!remaining) break;
    assert(rig.controller.state() == MotionState::Moving);
    rig.untilRequest();
    assert(rig.arm.selectedPart == -1); // Release happened before next camera Q.
  }
  assert(rig.scans.size() == 4 && rig.arm.requests.size() == 4);
  assert(rig.scans[1].target.classId == 1 && rig.scans[1].target.x10 == 179);
  assert(rig.scans[2].target.classId == 3 && rig.scans[2].target.x10 == -41);
  assert(rig.scans[0].target.x10 != rig.scans[1].target.x10);
  assert(rig.controller.state() == MotionState::Done && !rig.controller.outputsOn());
  assert(rig.controller.attempts() == 3 && rig.controller.clearedTargets() == 3);
  assert(rig.arm.releases == std::vector<unsigned>({1,1,3}));
  for (const auto& part : rig.world) assert(!part.present);
  report.scans = rig.scans;
  report.releasedClasses = rig.arm.releases;
  report.virtualElapsedMs = rig.now;
  report.commandedPoses = rig.arm.poses.size();
  report.ignoredDuplicates = rig.controller.staleReplies();
}

void corruptWireCannotMove(ExperimentReport& report) {
  Pipeline rig;
  rig.start();
  const size_t posesBefore = rig.arm.poses.size();
  std::string bad = rig.captureReply();
  assert(bad.size() > 6 && bad.back() == '\n');
  bad[bad.size()-2] = bad[bad.size()-2] == '0' ? '1' : '0'; // Corrupt CRC only.
  assert(rig.deliver(bad) == 0);
  assert(rig.controller.state() == MotionState::WaitScene && rig.controller.attempts() == 0);
  while (rig.controller.state() != MotionState::Fault && rig.now < 25000) rig.tick();
  assert(rig.controller.state() == MotionState::Fault);
  assert(rig.controller.error() == MotionError::CameraTimeout && !rig.controller.outputsOn());
  assert(rig.arm.requests.size() == 3); // Same-sequence retries, then stop.
  assert(rig.arm.releases.empty() && rig.arm.poses.size() == posesBefore);
  for (const auto& part : rig.world) assert(part.present);
  report.corruptParserErrors = rig.controllerParser.errors();
  report.corruptAdditionalPoses = rig.arm.poses.size()-posesBefore;
  assert(report.corruptParserErrors == 1);
}

void wrongCalibrationCannotMove(ExperimentReport& report) {
  Pipeline rig;
  rig.start();
  const size_t posesBefore = rig.arm.poses.size();
  // Unlike the previous case this has a correct CRC, valid field types and
  // valid coordinates. It must be rejected by the controller's calibration gate.
  const std::string mismatch = rig.captureReply(true);
  assert(rig.deliver(mismatch) == 1);
  assert(rig.controller.state() == MotionState::Fault);
  assert(rig.controller.error() == MotionError::CalibrationMismatch && !rig.controller.outputsOn());
  assert(rig.controller.attempts() == 0 && rig.arm.releases.empty());
  assert(rig.arm.poses.size() == posesBefore && rig.controllerParser.errors() == 0);
  for (const auto& part : rig.world) assert(part.present);
  report.wrongCalibrationAdditionalPoses = rig.arm.poses.size()-posesBefore;
}

bool writeReport(const char* path, const ExperimentReport& report) {
  FILE* file = fopen(path,"wb");
  if (!file) return false;
  fprintf(file,"{\n  \"test\": \"synthetic_camera_to_motion_pipeline\",\n");
  fprintf(file,"  \"result\": \"PASS\",\n  \"synthetic_geometry\": true,\n  \"hardware_executed\": false,\n");
  fprintf(file,"  \"world_assumption\": \"part removed after commanded correct-bin open release pose\",\n");
  fprintf(file,"  \"frame\": {\"format\": \"RGB565BE\", \"width\": %u, \"height\": %u},\n",WIDTH,HEIGHT);
  fprintf(file,"  \"scans\": [\n");
  for (size_t i = 0; i < report.scans.size(); ++i) {
    const SceneRecord& record = report.scans[i];
    fprintf(file,"    {\"sequence\": %u, \"objects\": %u",unsigned(record.sequence),record.count);
    if (record.count) fprintf(file,", \"selected_class\": %u, \"x10\": %d, \"y10\": %d, \"pixels\": %u",
      record.target.classId,int(record.target.x10),int(record.target.y10),unsigned(record.target.pixels));
    fprintf(file,"}%s\n", i+1 == report.scans.size() ? "" : ",");
  }
  fprintf(file,"  ],\n  \"release_bins\": [%u, %u, %u],\n",
    report.releasedClasses[0],report.releasedClasses[1],report.releasedClasses[2]);
  fprintf(file,"  \"controller_final_state\": \"DONE\",\n  \"outputs_on_at_end\": false,\n");
  fprintf(file,"  \"virtual_elapsed_ms\": %u,\n  \"commanded_pose_count\": %zu,\n",
    unsigned(report.virtualElapsedMs),report.commandedPoses);
  fprintf(file,"  \"ignored_duplicate_replies\": %u,\n",unsigned(report.ignoredDuplicates));
  fprintf(file,"  \"corrupt_uart\": {\"parser_errors\": %u, \"additional_pose_commands\": %zu, \"final_error\": \"CAMERA_TIMEOUT\"},\n",
    unsigned(report.corruptParserErrors),report.corruptAdditionalPoses);
  fprintf(file,"  \"wrong_calibration\": {\"additional_pose_commands\": %zu, \"final_error\": \"CALIBRATION_MISMATCH\"}\n}\n",
    report.wrongCalibrationAdditionalPoses);
  const bool okay = !ferror(file);
  return fclose(file) == 0 && okay;
}
} // namespace

int main(int argc, char** argv) {
  if (argc != 1 && argc != 3) {
    fprintf(stderr,"Usage: test_pipeline [--report output.json]\n"); return 2;
  }
  if (argc == 3 && strcmp(argv[1],"--report")) return 2;
  ExperimentReport report;
  normalClosedLoop(report);
  corruptWireCannotMove(report);
  wrongCalibrationCannotMove(report);
  if (argc == 3 && !writeReport(argv[2],report)) {
    fprintf(stderr,"Could not write pipeline report\n"); return 3;
  }
  printf("pipeline: PASS; RGB565BE -> detector -> homography -> CRC UART -> motion; scans 3,2,1,0; bins 1,1,3; DONE; %u virtual ms; %zu pose commands\n",
         unsigned(report.virtualElapsedMs),report.commandedPoses);
  puts("pipeline: corrupted UART and mismatched calibration produced zero additional pose commands; SYNTHETIC SOFTWARE ONLY");
  return 0;
}
