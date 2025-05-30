// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "ContestManager.hpp"

ContestManager::ContestManager(const Contest _contest,
                               const Trace &trace_full,
                               const Trace &trace_triangle,
                               const Trace &trace_sprint,
                               bool predict_triangle) noexcept
  :contest(_contest),
   olc_sprint(trace_sprint),
   olc_fai(trace_triangle, predict_triangle),
   olc_classic(trace_full),
   olc_league(trace_sprint),
   dmst_quad(trace_full),
   xcontest_free(trace_full, false),
   xcontest_triangle(trace_triangle, predict_triangle, false),
   dhv_xc_free(trace_full, true),
   dhv_xc_triangle(trace_triangle, predict_triangle, true),
   sis_at(trace_full),
   weglide_distance(trace_full),
   weglide_fai(trace_triangle, predict_triangle),
   weglide_or(trace_full),
   charron_small(trace_triangle, false),
   charron_large(trace_triangle, true)
{
  Reset();
}

void
ContestManager::SetIncremental(bool incremental) noexcept
{
  olc_sprint.SetIncremental(incremental);
  olc_fai.SetIncremental(incremental);
  olc_classic.SetIncremental(incremental);
  dmst_quad.SetIncremental(incremental);
  xcontest_free.SetIncremental(incremental);
  xcontest_triangle.SetIncremental(incremental);
  dhv_xc_free.SetIncremental(incremental);
  dhv_xc_triangle.SetIncremental(incremental);
  sis_at.SetIncremental(incremental);
  weglide_distance.SetIncremental(incremental);
  weglide_fai.SetIncremental(incremental);
  weglide_or.SetIncremental(incremental);
  charron_small.SetIncremental(incremental);
  charron_large.SetIncremental(incremental);
}

void
ContestManager::SetPredicted(const TracePoint &predicted) noexcept
{
  if (olc_classic.SetPredicted(predicted)) {
    olc_league.Reset();
    olc_plus.Reset();

    if (contest == Contest::OLC_CLASSIC || contest == Contest::OLC_LEAGUE ||
        contest == Contest::OLC_PLUS)
      stats.Reset();
  }

  if (dmst_quad.SetPredicted(predicted) &&
      contest == Contest::DMST)
    stats.Reset();
  
  if (weglide_distance.SetPredicted(predicted)) {
    weglide_fai.Reset();
    weglide_or.Reset();
    weglide_free.Reset();

    if (contest == Contest::WEGLIDE_DISTANCE ||
        contest == Contest::WEGLIDE_FAI ||
        contest == Contest::WEGLIDE_OR ||
        contest == Contest::WEGLIDE_FREE)
      stats.Reset();
  } else {
    if (weglide_fai.SetPredicted(predicted) && contest == Contest::WEGLIDE_FAI)
      stats.Reset();
    if (weglide_or.SetPredicted(predicted) && contest == Contest::WEGLIDE_OR)
      stats.Reset();
  }

  const bool reset_charron_large = charron_large.SetPredicted(predicted);
  const bool reset_charron_small = charron_small.SetPredicted(predicted);
  if (reset_charron_large)
    charron_small.Reset();

  if ((reset_charron_large || reset_charron_small) && contest == Contest::CHARRON)
      stats.Reset();
}

void
ContestManager::SetHandicap(unsigned handicap) noexcept
{
  olc_sprint.SetHandicap(handicap);
  olc_fai.SetHandicap(handicap);
  olc_classic.SetHandicap(handicap);
  olc_league.SetHandicap(handicap);
  olc_plus.SetHandicap(handicap);
  dmst_quad.SetHandicap(handicap);
  xcontest_free.SetHandicap(handicap);
  xcontest_triangle.SetHandicap(handicap);
  dhv_xc_free.SetHandicap(handicap);
  dhv_xc_triangle.SetHandicap(handicap);
  sis_at.SetHandicap(handicap);
  weglide_free.SetHandicap(handicap);
  weglide_distance.SetHandicap(handicap);
  weglide_fai.SetHandicap(handicap);
  weglide_or.SetHandicap(handicap);
  charron_small.SetHandicap(handicap);
  charron_large.SetHandicap(handicap);
}

static bool
RunContest(AbstractContest &_contest,
           ContestResult &result, ContestTraceVector &solution,
           bool exhaustive) noexcept
{
  // run solver, return immediately if further processing is required
  // by subsequent calls
  SolverResult r = _contest.Solve(exhaustive);
  if (r != SolverResult::VALID)
    return false;

  // if no improved solution was found, must have finished processing
  // with invalid data
  result = _contest.GetBestResult();

  // solver finished and improved solution was found.  save solution
  // and retrieve new trace.

  solution = _contest.GetBestSolution();

  return true;
}

bool
ContestManager::UpdateIdle(bool exhaustive) noexcept
{
  bool retval = false;

  switch (contest) {
  case Contest::NONE:
    break;

  case Contest::OLC_SPRINT:
    retval = RunContest(olc_sprint, stats.result[0],
                        stats.solution[0], exhaustive);
    break;

  case Contest::OLC_FAI:
    retval = RunContest(olc_fai, stats.result[0],
                        stats.solution[0], exhaustive);
    break;

  case Contest::OLC_CLASSIC:
    retval = RunContest(olc_classic, stats.result[0],
                        stats.solution[0], exhaustive);
    break;

  case Contest::OLC_LEAGUE:
    retval = RunContest(olc_classic, stats.result[1],
                        stats.solution[1], exhaustive);

    olc_league.Feed(stats.solution[1]);

    retval |= RunContest(olc_league, stats.result[0],
                         stats.solution[0], exhaustive);
    break;

  case Contest::OLC_PLUS:
    retval = RunContest(olc_classic, stats.result[0],
                        stats.solution[0], exhaustive);

    retval |= RunContest(olc_fai, stats.result[1],
                         stats.solution[1], exhaustive);

    if (retval) {
      olc_plus.Feed(stats.result[0], stats.solution[0],
                    stats.result[1], stats.solution[1]);

      RunContest(olc_plus, stats.result[2],
                 stats.solution[2], exhaustive);
    }

    break;

  case Contest::DMST:
    retval = RunContest(dmst_quad, stats.result[0],
                        stats.solution[0], exhaustive);
    break;

  case Contest::XCONTEST:
    retval = RunContest(xcontest_free, stats.result[0],
                        stats.solution[0], exhaustive);
    retval |= RunContest(xcontest_triangle, stats.result[1],
                         stats.solution[1], exhaustive);
    break;

  case Contest::DHV_XC:
    retval = RunContest(dhv_xc_free, stats.result[0],
                        stats.solution[0], exhaustive);
    retval |= RunContest(dhv_xc_triangle, stats.result[1],
                         stats.solution[1], exhaustive);
    break;

  case Contest::SIS_AT:
    retval = RunContest(sis_at, stats.result[0],
                        stats.solution[0], exhaustive);
    break;

  case Contest::WEGLIDE_FREE:
    retval = RunContest(weglide_distance, stats.result[0],
                        stats.solution[0], exhaustive);

    retval |= RunContest(weglide_fai, stats.result[1],
                         stats.solution[1], exhaustive);

    retval |= RunContest(weglide_or, stats.result[2],
                         stats.solution[2], exhaustive);

    if (retval) {
      weglide_free.Feed(stats.result[0], stats.solution[0],
                        stats.result[1], stats.solution[1],
                        stats.result[2], stats.solution[2]);

      RunContest(weglide_free, stats.result[3],
                 stats.solution[3], exhaustive);
    }
    break;

  case Contest::WEGLIDE_DISTANCE:
    retval = RunContest(weglide_distance, stats.result[0],
                        stats.solution[0], exhaustive);
    break;

  case Contest::WEGLIDE_FAI:
    retval = RunContest(weglide_fai, stats.result[0],
                        stats.solution[0], exhaustive);
    break;

  case Contest::WEGLIDE_OR:
    retval = RunContest(weglide_or, stats.result[0],
                        stats.solution[0], exhaustive);
    break;

  case Contest::CHARRON:
    retval = RunContest(charron_large, stats.result[0],
                        stats.solution[0], exhaustive);

    if (!retval) {
      retval = RunContest(charron_small, stats.result[0],
                          stats.solution[0], exhaustive);
    }
    break;

  };

  return retval;
}

void
ContestManager::Reset() noexcept
{
  stats.Reset();
  olc_sprint.Reset();
  olc_fai.Reset();
  olc_classic.Reset();
  olc_league.Reset();
  olc_plus.Reset();
  dmst_quad.Reset();
  xcontest_free.Reset();
  xcontest_triangle.Reset();
  dhv_xc_free.Reset();
  dhv_xc_triangle.Reset();
  sis_at.Reset();
  weglide_free.Reset();
  weglide_distance.Reset();
  weglide_fai.Reset();
  weglide_or.Reset();
  charron_small.Reset();
  charron_large.Reset();
}

/*

- SearchPointVector find self intersections (for OLC-FAI)
  -- eliminate bad candidates
  -- remaining candidates are potential finish points

- Possible use of convex reduction for approximate solution to triangle

- Specialised thinning routine; store max/min altitude etc
*/
