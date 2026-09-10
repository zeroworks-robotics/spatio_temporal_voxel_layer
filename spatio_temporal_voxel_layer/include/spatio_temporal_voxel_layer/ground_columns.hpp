#pragma once
// Ground segmentation for ORGANIZED depth clouds. Dependency-free (no ROS/PCL/Eigen)
// so it can be unit-tested in isolation.
//
// What it is for here. This layer gates obstacle height with a fixed band measured in
// height_filter_frame: min_obstacle_height .. max_obstacle_height, 0.12 .. 1.20 on this
// robot. A fixed band assumes the floor is at a fixed height, and on a ramp it is not --
// 3 m up a 10-degree ramp the ramp's own surface stands 0.5 m above base_link, so the
// floor the robot is about to drive on is marked lethal and the ramp becomes a wall.
// Raising min_obstacle_height to cover the ramp would blind the robot to real low
// obstacles everywhere else. The band has to be measured from the LOCAL GROUND instead,
// and that is what this finds.
//
// Why a column walk rather than a plane fit or Patchwork++. The five depth cameras
// deliver ORGANIZED clouds (160 x 100 on this robot): pixel (u, v) and (u, v-1) are
// already adjacent in space, for free. A concentric-zone partition exists to recover
// neighbourhood among points that HAVE none, which is a spinning lidar's problem, not
// this one -- and a single camera's wedge leaves most of its bins empty. A single plane
// fit is worse still: the flat-to-ramp transition is exactly where one plane cannot fit,
// and that transition is the case this has to get right.
//
// So this walks each image COLUMN instead. For a downward-tilted camera a column is a
// section through the ground running from near (bottom of the image) to far (top), so
// consecutive pixels down a column are consecutive samples along the floor. Two tests
// decide each step, and between them they are what makes slopes work:
//
//   1. Local grade. The angle between consecutive accepted points, which follows a
//      ramp exactly as well as it follows a flat floor -- the ground is never assumed
//      to be a plane, or level, or at any particular height. This is the classic
//      organized-range-image ground removal (Bogoslavskyi & Stachniss; the same idea
//      LeGO-LOAM uses on its range image).
//   2. Running height. Grade alone drifts: a long run of small accepted rises adds up,
//      and a wall met edge-on can be walked up a step at a time. Each accepted sample
//      is therefore also required to sit near the ground height PREDICTED by extending
//      the recent accepted run, which bounds the drift without re-imposing a plane.
//
// The output is per-pixel ground / non-ground plus, for each column, the ground profile
// as (range, z) samples, so a caller can gate a point on its height above the ground
// beneath it rather than above the robot.
//
// Frame: points must be in a frame whose z is up and whose z ~ 0 is the plane the robot
// stands on (base_link on this robot). Nothing here reads x/y orientation, so the
// camera's yaw and tilt do not matter; its ROLL does, mildly -- a column is assumed to
// run roughly along the ground's steepest direction, which is what an unrolled camera
// gives.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace ground_seg
{

struct GroundColumnsConfig
{
	// --- seeding ---------------------------------------------------------------
	// A column starts from the pixels nearest the robot, which for a downward-tilted
	// camera are at the bottom of the image and are floor unless the robot is nose to
	// nose with something. A sample seeds the run when it lies within this band of the
	// robot's own standing plane (z = 0). Wide enough for the near floor to be found
	// under sensor noise and a little pitch; tight enough that a box right in front of
	// the camera cannot become the seed.
	float seed_z_tol = 0.10f;
	// Do not seed beyond this range. The seed is trusted more than later samples, and
	// that trust is only justified where the geometry says floor is nearly certain.
	float seed_max_range = 2.0f;

	// --- grade test (1) ---------------------------------------------------------
	// Maximum grade between consecutive ground samples, in degrees. A step steeper than
	// this ends the ground run. This is the drivable-slope limit, not a noise
	// parameter: raise it and the segmenter will happily walk up a ramp the chassis
	// cannot climb.
	float max_grade_deg = 20.0f;
	// Minimum range separation from the anchor for the grade test to be applied at all.
	// Below it the separation is comparable to depth noise and the grade is meaningless,
	// so only the height test (2) judges the sample.
	float min_grade_span = 0.05f;

	// --- height test (2) --------------------------------------------------------
	// How far a sample may sit from the height predicted by extending the recent
	// accepted run. Absorbs depth noise, which grows with distance, hence the second
	// term: tolerance is height_tol + height_tol_per_m * range.
	//
	// This is the parameter to fit first on real data, and it is not a safety margin to
	// be minimised -- it is a noise budget, and setting it below the sensor's actual
	// noise silently shreds the floor. Measured on synthetic scenes at the fdown rig,
	// flat-floor ground recall against per-point sigma: 0.06 holds 100% at sigma 2 mm
	// but only 86% at 1 cm and 37% at 2 cm, while 0.08 holds 98% at 1 cm. A Gemini 335
	// at several metres is well past 1 cm, so the default is set for that and should be
	// re-fitted per camera once real clouds are available.
	float height_tol = 0.10f;
	float height_tol_per_m = 0.03f;
	// How far BACK IN RANGE the prediction is anchored, in metres. Deliberately a
	// distance and not a sample count: on a vertical face many samples pile up at one
	// range, so a count-based window slides up the face and the anchor climbs with it,
	// which is precisely the drift this is here to stop. A distance-based anchor cannot
	// move up a face at all, because no sample on it is ever this much farther out.
	// Short enough that a ramp is followed, long enough to span depth noise.
	float slope_anchor_span = 0.10f;
	// Slope carried into the prediction is clamped to this, so a mis-accepted sample
	// cannot launch the prediction off the floor. Slightly above max_grade_deg: the
	// grade test already rejects steeper single steps, and leaving headroom keeps a
	// legitimate ramp from being clipped as it is entered.
	float max_predict_grade_deg = 25.0f;

	// --- run continuity ---------------------------------------------------------
	// A ground run survives this many consecutive rejected samples before it is
	// considered ended. Depth cameras drop pixels on dark or glossy floor, and a run
	// that ended at every dropout would report a cliff at every scuff mark.
	int max_gap = 3;
	// A gap in RANGE longer than this ends the run regardless of max_gap: something
	// occluded the floor, or the floor is not there. Distinct from max_gap, which
	// counts pixels; this bounds the physical distance being bridged.
	float max_gap_range = 0.5f;

	// --- input gating -----------------------------------------------------------
	// Ignore samples outside this range band. The near limit drops the camera's own
	// housing; the far limit is where depth noise exceeds height_tol anyway.
	float min_range = 0.15f;
	float max_range = 6.0f;
	// Points above this height cannot be floor under any slope and are not offered to
	// the walk at all, so a low ceiling or a table top cannot terminate a column early.
	float max_ground_z = 1.0f;
	// Points this far BELOW the running ground are holes, not floor and not obstacles.
	// Reported separately so a drop-off does not become a positive obstacle.
	float hole_depth = 0.15f;
};

// What a single pixel was judged to be.
enum class GroundClass : unsigned char
{
	INVALID = 0,  // no depth, or outside the configured range / height gates
	GROUND = 1,   // part of an accepted ground run
	OBSTACLE = 2, // above the local ground by more than the tolerance
	HOLE = 3,     // below the local ground by more than hole_depth
};

// One 3D sample. Deliberately not Eigen: this header stays dependency-free, and the
// caller converts once while it is already iterating the cloud.
struct GroundPoint
{
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	bool valid = false; // false for a NaN / zero-depth pixel
};

struct GroundColumnsResult
{
	// Per-pixel class, row-major, width * height. Same indexing as the input.
	std::vector<GroundClass> classes;
	// Per-column ground profile as (range, z), ascending in range -- the input
	// analyze_beam() takes. Indexed by image column.
	std::vector<std::vector<std::pair<float, float>>> column_ground;
	int ground_count = 0;
	int obstacle_count = 0;
	int hole_count = 0;
	// Columns that never found a seed. A high fraction means the camera is not looking
	// at floor it can trust -- pitched up, blocked, or standing on a drop.
	int columns_unseeded = 0;
};

// Ground segmentation over an organized grid of points, row-major, width * height.
// `pts` must hold width * height entries; invalid pixels are marked valid = false.
//
// Rows are walked from the LAST row towards the first, which for a downward-tilted
// camera is near-to-far. A camera mounted upside down should hand in its rows already
// flipped rather than have a flag here decide it.
inline GroundColumnsResult segment_ground_columns(
	const std::vector<GroundPoint> &pts, int width, int height,
	const GroundColumnsConfig &cfg)
{
	GroundColumnsResult out;
	if (width <= 0 || height <= 0 ||
		pts.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
		return out;

	out.classes.assign(pts.size(), GroundClass::INVALID);
	out.column_ground.assign(static_cast<std::size_t>(width), {});

	const float max_grade_tan = std::tan(cfg.max_grade_deg * 3.14159265f / 180.0f);
	const float max_predict_tan = std::tan(cfg.max_predict_grade_deg * 3.14159265f / 180.0f);

	// Reused across columns so a steady-state frame does no allocation in the loop.
	std::vector<std::pair<float, float>> run;
	run.reserve(static_cast<std::size_t>(height));

	for (int u = 0; u < width; ++u)
	{
		run.clear();
		bool seeded = false;
		int gap = 0;
		float last_range = 0.0f;

		for (int v = height - 1; v >= 0; --v)
		{
			const std::size_t idx = static_cast<std::size_t>(v) * static_cast<std::size_t>(width) +
									static_cast<std::size_t>(u);
			const GroundPoint &p = pts[idx];
			if (!p.valid)
				continue;

			const float range = std::sqrt(p.x * p.x + p.y * p.y);
			if (range < cfg.min_range || range > cfg.max_range)
				continue;
			if (p.z > cfg.max_ground_z)
			{
				// Too high to be floor at any slope. Left OBSTACLE rather than INVALID:
				// it is a real return, and the caller's ceiling gate -- not this one --
				// decides whether the robot can drive under it.
				out.classes[idx] = GroundClass::OBSTACLE;
				++out.obstacle_count;
				continue;
			}

			if (!seeded)
			{
				// Seed on the near floor. Anything else near the bottom of the frame is
				// an obstacle standing on floor we have not established yet, so it is
				// left unclassified rather than guessed at.
				if (range <= cfg.seed_max_range && std::fabs(p.z) <= cfg.seed_z_tol)
				{
					seeded = true;
					run.emplace_back(range, p.z);
					out.classes[idx] = GroundClass::GROUND;
					++out.ground_count;
					last_range = range;
				}
				continue;
			}

			// --- predict the ground height at this range from the recent run --------
			// Anchored a fixed DISTANCE back along the run, not at the previous sample
			// and not a fixed number of samples back. Predicting from the previous
			// sample re-zeroes the error at every acceptance, so a surface the run
			// should never have climbed gets climbed anyway, one small step at a time;
			// a vertical face is exactly that surface, since consecutive pixels up a
			// wall sit ~1 cm apart in z and ~0 apart in range, each step comfortably
			// inside the tolerance, and with dr = 0 the grade test cannot fire either.
			// A sample-count window does not fix it: accepted face samples enter the
			// window and the anchor climbs the wall with them. Anchoring by range does,
			// because no sample on a vertical face is ever farther out than the floor
			// below it, so the anchor stays on that floor and the deviation accumulates
			// until the face is rejected.
			const int n = static_cast<int>(run.size());
			const std::pair<float, float> &tail = run[static_cast<std::size_t>(n - 1)];
			// Two anchors, each a span farther back along the run. The slope is measured
			// between them rather than up to the run's end, because the run's end is
			// where a wrongly accepted sample sits: on a vertical face every sample
			// shares one range, so a slope taken to the end reads that face's height
			// over the span as grade, which then lifts the prediction and lets the rest
			// of the face in. Neither anchor can advance onto a face, since advancing
			// requires range that a face does not provide.
			int a = n - 1;
			while (a > 0 && tail.first - run[static_cast<std::size_t>(a)].first < cfg.slope_anchor_span)
				--a;
			const std::pair<float, float> &anchor = run[static_cast<std::size_t>(a)];
			int b = a;
			while (b > 0 && anchor.first - run[static_cast<std::size_t>(b)].first < cfg.slope_anchor_span)
				--b;
			const std::pair<float, float> &prev = run[static_cast<std::size_t>(b)];
			float slope = 0.0f;
			{
				const float dr = anchor.first - prev.first;
				if (dr > 1e-3f)
					slope = (anchor.second - prev.second) / dr;
			}
			slope = std::max(-max_predict_tan, std::min(max_predict_tan, slope));

			// Gap bookkeeping measures from the run's end; the two tests below measure
			// from the anchor.
			const float dr_pred = range - tail.first;
			const float dr_anchor = range - anchor.first;
			const float z_pred = anchor.second + slope * dr_anchor;
			const float tol = cfg.height_tol + cfg.height_tol_per_m * range;

			// A sample well below the predicted floor is a hole. Judged before the grade
			// test on purpose: the drop into a stairwell is steep, and calling it an
			// obstacle would put a phantom wall at the top of the stairs.
			if (p.z < z_pred - cfg.hole_depth)
			{
				out.classes[idx] = GroundClass::HOLE;
				++out.hole_count;
				continue;
			}

			// --- the two tests -----------------------------------------------------
			bool accept = std::fabs(p.z - z_pred) <= tol;
			if (accept && dr_anchor >= cfg.min_grade_span)
			{
				// Grade measured from the anchor, not from the previous sample. Between
				// consecutive pixels the range separation is often a few millimetres,
				// where depth noise alone spans any grade you care to name and the test
				// says nothing; over the anchor span it is a real measurement. It is
				// also what catches a vertical face outright, since the face's whole
				// height is divided by the span back to the floor below it.
				const float grade = std::fabs(p.z - anchor.second) / dr_anchor;
				accept = grade <= max_grade_tan;
			}

			// A run cannot bridge an arbitrary distance: past max_gap dropped samples,
			// or a physical jump longer than max_gap_range, the floor beyond is a new
			// surface rather than a continuation of this one.
			if (accept && (gap > cfg.max_gap || dr_pred > cfg.max_gap_range))
				accept = false;

			if (accept)
			{
				run.emplace_back(range, p.z);
				out.classes[idx] = GroundClass::GROUND;
				++out.ground_count;
				gap = 0;
				last_range = range;
			}
			else
			{
				out.classes[idx] = GroundClass::OBSTACLE;
				++out.obstacle_count;
				++gap;
			}
		}

		if (!seeded)
			++out.columns_unseeded;
		out.column_ground[static_cast<std::size_t>(u)] = run;
		(void)last_range;
	}

	return out;
}

}  // namespace ground_seg
