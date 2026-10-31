#!/usr/bin/env python3
"""Patch only the frozen GridMap .cpp; copy delivery_fast_query.hpp beside it."""
from build_state_contract import Transformer


def transform_grid_map(text):
    t = Transformer(text)
    t.after('#include "plan_env/grid_map.h"', '\n#include "delivery_fast_query.hpp"')
    t.change('''  if (dynamic_point_overlay_kdtree_.radiusSearch(
          query, geometric_search_limit, candidate_indices,
          candidate_squared_distances) <= 0)''', '''  if (delivery_fast_query::candidates(
          this, dynamic_point_overlay_cloud_, dynamic_point_overlay_radii_,
          dynamic_point_overlay_sequence_,
          dynamic_point_overlay_refresh_time_.toNSec(),
          dynamic_point_overlay_kdtree_, query, geometric_search_limit,
          delivery_fast_query::enabled(), candidate_indices,
          candidate_squared_distances) <= 0)''')
    return t.finish()


transform = transform_grid_map
