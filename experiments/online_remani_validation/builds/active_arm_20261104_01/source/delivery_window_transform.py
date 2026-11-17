"""Extend only a slow solve's frozen CV copy to cover delivery latency."""
from native_collision_runtime import replace_once


def transform_coverage(text):
    text = replace_once(text, '#include "dual_timescale_probe.hpp"',
        '#include "dual_timescale_probe.hpp"\n#include "delivery_window.hpp"')
    text = replace_once(text,
        '  const auto& s=c->source;const auto& p=s.parameters;',
        '''  // Keep the measured source epoch and all point states fixed. The
  // extra interval is an explicit CV extrapolation for this planning copy,
  // not a claim that the live source buffer has longer validated coverage.
  const auto delivery_window=slow_delivery_window::choose(
    slow_delivery_window::requested(),dual_probe::request(),c->full_coverage,
    c->source.parameters.max_prediction);
  c->source.parameters.max_prediction=delivery_window.planning_max_s;
  const auto& s=c->source;const auto& p=s.parameters;
  dual_probe::emit("slow_delivery_window",dual_probe::Fields()
    .add("requested",slow_delivery_window::requested()?"true":"false")
    .add("enabled",delivery_window.enabled?"true":"false")
    .add("full_coverage",c->full_coverage?"true":"false")
    .add("original_source_sequence",s.snapshot.sequence)
    .add("original_source_stamp",s.snapshot.stamp.toSec())
    .add("original_source_refresh_epoch",s.refresh_epoch)
    .add("original_max_prediction_s",delivery_window.original_max_s)
    .add("original_end_epoch",s.snapshot.stamp.toSec()+delivery_window.original_max_s)
    .add("planning_max_prediction_s",delivery_window.planning_max_s)
    .add("planning_end_epoch",s.snapshot.stamp.toSec()+delivery_window.planning_max_s)
    .add("configured_delivery_allowance_s",slow_delivery_window::configured_allowance_s)
    .add("delivery_allowance_s",delivery_window.allowance_s)
    .add("optimizer_budget_reference_s",slow_delivery_window::optimizer_budget_reference_s)
    .add("requested_epoch",c->requested_epoch).add("optimizer_entry_epoch",c->entry_epoch)
    .str("model_assumption","fixed source states and epoch; CV extrapolation in planning copy only")
    .str("scope","shared slow soft/hard planning window; latest source and execution checks unchanged"));''')
    return replace_once(text,
        '    .add("maximum_prediction_s",p.max_prediction).add("guide_horizon_s",p.horizon)',
        '    .add("maximum_prediction_s",p.max_prediction).add("guide_horizon_s",p.horizon)\n'
        '    .add("original_source_max_prediction_s",delivery_window.original_max_s)\n'
        '    .add("original_source_end_epoch",s.snapshot.stamp.toSec()+delivery_window.original_max_s)\n'
        '    .add("delivery_allowance_s",delivery_window.allowance_s)')
