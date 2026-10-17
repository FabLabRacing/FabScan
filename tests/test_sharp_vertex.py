"""Headless tests for vertex geometry, execution state and real-controller handoff.

Run: python -m unittest discover -s tests -v
Synthetic frames exercise the real connected-boundary extractor and planner.
No LinuxCNC connection or machine commands are used.
"""
from contextlib import nullcontext
from dataclasses import replace
import csv
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from fabscan.path_belief import BeliefState, ContextResult, PathBeliefEstimator
from fabscan.real_belief_follow import _controller_move_from_plan, _log_vertex_events
from fabscan.real_belief_follow import _m6_step_impl, RealBeliefSession, _build_belief, _record_executed_belief_pose
from fabscan.sharp_vertex import SharpVertex, VertexCommitment, find_sharp_vertex, EXIT_DISTANCE
from fabscan.virtual_planner import ConnectedPathPlanner, INTEGRATION_VERSION


PPI = 800.0
CALIBRATION = {
    "matrix_pixel_to_machine": [[-1/PPI, 0], [0, -1/PPI]],
    "matrix_machine_to_pixel": [[-PPI, 0], [0, -PPI]],
}
SETTINGS = {"line_mode": "Edge near center", "line_search_px": 400,
            "follow_step": 0.030, "follow_max_correct": 0.003}


def belief(camera, step, tangent=(1, 0), measurement=None):
    x, y = camera if measurement is None else measurement
    return BeliefState(
        step_id=step, step_index=step, measurement_valid=True,
        measurement_x=float(x), measurement_y=float(y), measured_heading_degrees=0,
        measured_confidence=85, estimate_x=float(x), estimate_y=float(y),
        tangent_x=tangent[0], tangent_y=tangent[1], tangent_degrees=0,
        raw_curvature=0, curvature=0, curvature_trend=0, implied_radius=0,
        usable_lookahead=0.4, prediction_horizon=0.2, belief_confidence=85,
        uncertainty=0, position_innovation=0, heading_innovation_degrees=0,
        context=ContextResult(found=True), recent_measurements=(), predicted_points=(), reason="test",
    )


def render(camera, polygon):
    frame = np.full((600, 800, 3), 255, np.uint8)
    points = np.rint((np.asarray(polygon)-camera)*PPI + [400, 300]).astype(np.int32)
    cv2.fillPoly(frame, [points], (0, 0, 0))
    return frame


def leg_path(turn=90.0):
    a = np.deg2rad(turn)
    incoming = np.column_stack([np.linspace(-0.20, 0, 81), np.zeros(81)])
    outgoing = np.linspace(0.0025, 0.20, 80)[:, None] * [np.cos(a), np.sin(a)]
    return np.vstack([incoming, outgoing])


class GeometryTests(unittest.TestCase):
    def test_sharp_corners_both_directions_and_rotations(self):
        for turn in (-120, -90, -60, 60, 90, 120):
            for rotation in (0, 27, 90):
                with self.subTest(turn=turn, rotation=rotation):
                    a = np.deg2rad(rotation)
                    rot = np.asarray([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
                    vertex, reason = find_sharp_vertex(leg_path(turn) @ rot.T + [12, 7], pixel_size=1/PPI)
                    self.assertIsNotNone(vertex, reason)
                    np.testing.assert_allclose(vertex.point, [12, 7], atol=0.002)
                    self.assertAlmostEqual(vertex.turn_degrees, abs(turn), delta=1)

    def test_smooth_radii_and_straight_edges_do_not_commit(self):
        paths = [np.column_stack([np.linspace(0, .45, 100), np.zeros(100)])]
        for radius in (0.05, 0.10, 0.25, 0.50, 2.5):
            angles = np.linspace(-np.pi/2, 0, 200)
            arc = np.column_stack([radius*np.cos(angles), radius*np.sin(angles)])
            # Straight -> quarter circle -> straight, tangent continuous.
            paths.append(np.vstack([
                np.column_stack([np.linspace(-.15, 0, 60), np.full(60, -radius)]),
                arc, np.column_stack([np.full(60, radius), np.linspace(0, .15, 60)])]))
        for path in paths:
            vertex, reason = find_sharp_vertex(path, pixel_size=1/PPI)
            self.assertIsNone(vertex, reason)

    def test_small_edge_defects_do_not_become_vertices(self):
        x = np.linspace(0, .4, 200)
        y = .003 * np.sin(x * 200)
        y[99:103] += .006
        vertex, _ = find_sharp_vertex(np.column_stack([x, y]), pixel_size=1/PPI)
        self.assertIsNone(vertex)


class StateTests(unittest.TestCase):
    def setUp(self):
        self.state = VertexCommitment()
        self.v = SharpVertex((0, 0), (1, 0), (0, 1), 90, 0)
        self.camera = np.asarray([-.12, 0])

    def observe(self, vertex=None, observation_id=1):
        self.state.observe(vertex or self.v, camera=self.camera, observation_id=observation_id,
                           pixel_size=1/PPI, max_cross_track=.018)

    def test_same_frame_cannot_confirm_or_advance_arrival(self):
        self.observe()
        self.observe()
        self.assertEqual(self.state.state, "CANDIDATE")
        self.observe(observation_id=2)
        self.assertEqual(self.state.state, "APPROACH")
        for i in range(3, 10):
            self.observe(replace(self.v, point=(.05, .05)), i)
        self.assertEqual(self.state.vertex.point, (0, 0))
        self.assertEqual(self.state.state, "APPROACH")
        self.state.record_executed_pose(np.asarray([0, 0]))
        self.assertEqual(self.state.state, "OUTGOING")
        self.state.record_executed_pose(np.asarray([0, EXIT_DISTANCE]))
        self.assertEqual(self.state.state, "FOLLOW")
        self.assertEqual([e['vertex_event'] for e in self.state.drain_events()],
                         ['CANDIDATE', 'COMMITTED', 'VERTEX_REACHED', 'OUTGOING_LATCHED', 'RESUMED'])

    def test_inconsistent_observation_restarts_confirmation(self):
        self.observe()
        self.observe(replace(self.v, point=(.03, 0)), 2)
        self.assertEqual(self.state.state, "CANDIDATE")
        self.assertEqual(self.state.confirmations, 1)
        self.state.hold("missing boundary", self.camera)
        self.assertEqual(self.state.state, "FOLLOW")


class PlannerTests(unittest.TestCase):
    def new_planner(self, direction=(1, 0), camera=(-.20, 0)):
        p = ConnectedPathPlanner(CALIBRATION, SETTINGS)
        p.travel_unit = np.asarray(direction, dtype=float)
        p.record_executed_pose(*camera)
        return p

    def run_corner(self, sign=1, rotation=0, delayed=False, invert=False):
        a = np.deg2rad(rotation)
        rot = np.asarray([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        polygon = np.asarray([[-1, 0], [0, 0], [0, sign], [-1, sign]]) @ rot.T
        camera = np.asarray([-.20, 0]) @ rot.T
        direction = np.asarray([1, 0]) @ rot.T
        planner = self.new_planner(direction, camera)
        events = []
        committed = None
        saw_vertex = False
        for i in range(35):
            frame_camera = camera - .01 * direction if delayed else camera.copy()
            # Simulate a lagging tangent even after the corner. The latch must win.
            b = belief(camera, i, direction)
            frame = render(frame_camera, polygon)
            if invert:
                frame = 255 - frame
            plan = planner.plan(frame, camera_x=camera[0], camera_y=camera[1],
                                frame_camera_x=frame_camera[0], frame_camera_y=frame_camera[1], belief=b)
            self.assertTrue(plan.accepted, f"step {i}: {plan.decision_reason}")
            self.assertLessEqual(plan.move_length, .033 + 1e-9)
            if plan.motion_mode == "vertex_approach":
                point = (plan.vertex_diagnostics['vertex_x'], plan.vertex_diagnostics['vertex_y'])
                if committed is None:
                    committed = point
                self.assertEqual(point, committed)
                # No outgoing component before arriving at the vertex (raster tolerance).
                self.assertLess(abs(float((camera @ rot)[1])), .0031)
            mx, my, fields = _controller_move_from_plan(SimpleNamespace(), plan, *camera) if plan.motion_mode.startswith('vertex_') else (plan.move_x, plan.move_y, {})
            camera = camera + [mx, my]
            planner.record_executed_pose(*camera)
            transitions = planner.vertex.drain_events()
            events.extend(transitions)
            if any(e['vertex_event'] == 'VERTEX_REACHED' for e in transitions):
                saw_vertex = True
                self.assertLess(np.linalg.norm(camera), .0031)
            if any(e['vertex_event'] == 'RESUMED' for e in transitions):
                self.assertTrue(saw_vertex)
                np.testing.assert_allclose(planner.travel_unit, np.asarray([0, sign]) @ rot.T, atol=.03)
                # Exercise the first ordinary plan after releasing the latch.
                next_frame = render(camera, polygon)
                if invert:
                    next_frame = 255 - next_frame
                next_plan = planner.plan(next_frame, camera_x=camera[0], camera_y=camera[1], belief=belief(camera, i+1, direction))
                self.assertTrue(next_plan.accepted, next_plan.decision_reason)
                self.assertEqual(next_plan.motion_mode, 'boundary_follow')
                return planner, events
        self.fail(f"corner did not complete: {planner.vertex.state}; events {events}")

    def test_full_boundary_to_vertex_to_outgoing(self):
        for sign in (-1, 1):
            for rotation in (0, 27, 90):
                with self.subTest(sign=sign, rotation=rotation):
                    self.run_corner(sign, rotation)

    def test_frame_pose_is_separate_from_current_execution_pose(self):
        self.run_corner(delayed=True)

    def test_concave_boundary_uses_the_same_vertex_transition(self):
        self.run_corner(invert=True)

    def test_four_consecutive_corners_with_small_physical_step_cap(self):
        from fabscan.camera_calibration import CameraCalibrationDialog
        polygon = np.asarray([[-.6, 0], [0, 0], [0, .45], [-.6, .45]])
        camera = np.asarray([-.25, 0])
        p = self.new_planner(camera=camera)
        controller = SimpleNamespace(
            _get_follow_deadband=lambda: .003, _get_follow_gain=lambda: .35,
            _get_follow_max_correct=lambda: .003,
            _apply_follow_correction_tuning=CameraCalibrationDialog._apply_follow_correction_tuning,
        )
        reached = []
        resumed = 0
        for step in range(240):
            plan = p.plan(render(camera, polygon), camera_x=camera[0], camera_y=camera[1],
                          belief=belief(camera, step, tuple(p.travel_unit)))
            self.assertTrue(plan.accepted, f'{step}: {plan.decision_reason}')
            mx, my, _ = _controller_move_from_plan(controller, plan, *camera)
            delta = np.asarray([mx, my])
            # Model the independent physical cap being smaller than Follow Step.
            delta *= min(1., .012 / np.linalg.norm(delta))
            camera += delta
            p.record_executed_pose(*camera)
            for event in p.vertex.drain_events():
                if event['vertex_event'] == 'VERTEX_REACHED':
                    reached.append([event['vertex_x'], event['vertex_y']])
                resumed += event['vertex_event'] == 'RESUMED'
            if resumed == 4:
                np.testing.assert_allclose(reached, [[0, 0], [0, .45], [-.6, .45], [-.6, 0]], atol=.002)
                return
        self.fail(f'only {resumed} vertices completed')

    def committed_setup(self):
        camera = np.asarray([-.15, 0])
        polygon = np.asarray([[-1, 0], [0, 0], [0, 1], [-1, 1]], dtype=float)
        p = self.new_planner(camera=camera)
        for step in (1, 2):
            plan = p.plan(render(camera, polygon), camera_x=camera[0], camera_y=camera[1], belief=belief(camera, step))
        self.assertEqual(plan.motion_mode, 'vertex_approach')
        return p, camera, polygon

    def test_detector_drift_does_not_retarget_committed_corner(self):
        p, camera, polygon = self.committed_setup()
        original = p.vertex.vertex
        frame_camera = camera - [.01, 0]
        plan = p.plan(render(frame_camera, polygon), camera_x=camera[0], camera_y=camera[1],
                      frame_camera_x=frame_camera[0], frame_camera_y=frame_camera[1],
                      belief=belief(camera, 3, measurement=camera + [0, .08]))
        self.assertTrue(plan.accepted, plan.decision_reason)
        self.assertEqual(p.vertex.vertex, original)
        self.assertEqual(plan.vertex_diagnostics['vertex_association_source'], 'committed_incoming')
        np.testing.assert_allclose([plan.vertex_diagnostics['vertex_association_x'],
                                    plan.vertex_diagnostics['vertex_association_y']], frame_camera, atol=.002)

    def test_boundary_gate_remains_in_force_during_commitment(self):
        p, camera, polygon = self.committed_setup()
        plan = p.plan(render(camera, polygon + [0, .08]), camera_x=camera[0], camera_y=camera[1],
                      belief=belief(camera, 3))
        self.assertFalse(plan.accepted)
        self.assertIn('beyond 40.00 px limit', plan.decision_reason)
        self.assertEqual(plan.motion_mode, 'vertex_approach')
        self.assertEqual(p.vertex.state, 'APPROACH')
        self.assertEqual(p.vertex.drain_events()[-1]['vertex_event'], 'HOLD')

    def test_retire_only_traversed_incoming_support(self):
        # The same notch is ignored behind the executed camera, but stops a
        # move when it lies ahead. Also cover an approach longer than .080 in.
        for camera_x, notch_x, accepted in ((-.027, -.08, True),
                                           (-.027, -.015, False),
                                           (-.15, -.12, False)):
            with self.subTest(camera_x=camera_x, notch_x=notch_x):
                p, _, polygon = self.committed_setup()
                camera = np.array([camera_x, 0.])
                p.record_executed_pose(*camera)
                frame = render(camera, polygon)
                left = int(round((notch_x-.009-camera_x)*PPI+400))
                right = int(round((notch_x+.009-camera_x)*PPI+400))
                cv2.rectangle(frame, (left, 299), (right, 311), (255,255,255), -1)
                plan = p.plan(frame, camera_x=camera[0], camera_y=camera[1], belief=belief(camera, 3))
                self.assertEqual(plan.accepted, accepted, plan.decision_reason)
                if not accepted:
                    self.assertIn('lost boundary support', plan.decision_reason)

    def test_outgoing_association_ignores_detector_and_retired_incoming(self):
        p, _, polygon = self.committed_setup()
        p.record_executed_pose(*p.vertex.vertex.point)
        camera = np.array(p.vertex.vertex.point) + np.array(p.vertex.vertex.outgoing)*.03
        p.record_executed_pose(*camera)
        frame = render(camera, polygon)
        # Remove incoming boundary well behind the reached vertex.
        cv2.rectangle(frame, (320, 275), (360, 287), (255,255,255), -1)
        plan = p.plan(frame, camera_x=camera[0], camera_y=camera[1],
                      belief=belief(camera, 3, measurement=camera+[.08,0]))
        self.assertTrue(plan.accepted, plan.decision_reason)
        self.assertEqual(plan.motion_mode, 'vertex_outgoing')
        self.assertEqual(plan.vertex_diagnostics['vertex_association_source'], 'committed_outgoing')
        # Losing the untraversed outgoing segment must still stop execution.
        cv2.rectangle(frame, (389, 307), (401, 323), (255,255,255), -1)
        plan = p.plan(frame, camera_x=camera[0], camera_y=camera[1], belief=belief(camera, 4))
        self.assertFalse(plan.accepted)
        self.assertIn('lost boundary support', plan.decision_reason)

    def test_disappearing_outgoing_leg_holds_without_retargeting(self):
        p, camera, polygon = self.committed_setup()
        original = p.vertex.vertex.point
        polygon[:, 0] += .03  # Incoming straight edge stays visible, corner moves.
        plan = p.plan(render(camera, polygon), camera_x=camera[0], camera_y=camera[1], belief=belief(camera, 3))
        self.assertFalse(plan.accepted)
        self.assertIn('lost boundary support', plan.decision_reason)
        self.assertEqual(p.vertex.vertex.point, original)

    def test_local_tip_change_keeps_fixed_target_and_completes_transition(self):
        p, camera, _ = self.committed_setup()
        original = p.vertex.vertex
        # Replace only the tip after commitment; its observed boundary no longer
        # passes through the ideal intersection, but joins the same straight legs.
        polygon = np.array([[-1,0],[-.012,0],[0,.012],[0,1],[-1,1]])
        saw_large_tip_distance = False
        for step in range(3, 20):
            plan = p.plan(render(camera, polygon), camera_x=camera[0], camera_y=camera[1],
                          belief=belief(camera, step, measurement=camera+[0,.08]))
            self.assertTrue(plan.accepted, plan.decision_reason)
            self.assertEqual(p.vertex.vertex, original)
            self.assertTrue(plan.vertex_diagnostics['vertex_corner_connected'])
            saw_large_tip_distance |= plan.vertex_diagnostics['vertex_tip_distance_px'] > 4.0
            camera = np.array([plan.approved_x, plan.approved_y])
            p.record_executed_pose(*camera)
            if p.vertex.state == 'FOLLOW':
                self.assertTrue(saw_large_tip_distance)
                return
        self.fail('local tip transition did not finish')

    def test_tip_change_outside_corner_envelope_holds(self):
        p, camera, _ = self.committed_setup()
        original = p.vertex.vertex
        polygon = np.array([[-1,0],[-.04,0],[0,.04],[0,1],[-1,1]])
        plan = p.plan(render(camera, polygon), camera_x=camera[0], camera_y=camera[1],
                      belief=belief(camera, 3))
        self.assertFalse(plan.accepted)
        self.assertEqual(p.vertex.vertex, original)
        self.assertEqual(p.vertex.state, 'APPROACH')

    def test_corner_requires_observed_connectivity_without_gap_bridging(self):
        p, camera, _ = self.committed_setup()
        point = np.array(p.vertex.vertex.point)
        samples = np.array([point-[.04,0], point, point+[0,.04]])
        pixels = np.rint((samples-camera)*PPI+[400,300]).astype(np.int32)
        mask = np.zeros((600,800), np.uint8)
        cv2.polylines(mask, [pixels], False, 255, 1)
        self.assertEqual(p._committed_corner_support(mask, camera), '')
        x, y = pixels[1]
        mask[y-1:y+2, x-1:x+2] = 0
        self.assertIn('disconnected', p._committed_corner_support(mask, camera))
        self.assertFalse(p._association_diagnostics['vertex_corner_connected'])
        mask[:] = 0
        self.assertIn('lost', p._committed_corner_support(mask, camera))

    def test_failed_execution_does_not_latch_and_reset_clears_state(self):
        p, camera, polygon = self.committed_setup()
        for step in range(3, 6):
            p.plan(render(camera, polygon), camera_x=camera[0], camera_y=camera[1], belief=belief(camera, step))
        self.assertEqual(p.vertex.state, 'APPROACH')
        p.reset()
        self.assertEqual(p.vertex.state, 'FOLLOW')
        self.assertEqual(p.vertex.drain_events(), [])

    def test_minimum_follow_step_does_not_split_into_an_unusable_move(self):
        p, camera, polygon = self.committed_setup()
        p.follow_step = .005
        camera = np.asarray(p.vertex.vertex.point) - [.0058, 0]
        p.record_executed_pose(*camera)
        plan = p.plan(render(camera, polygon), camera_x=camera[0], camera_y=camera[1], belief=belief(camera, 3))
        self.assertTrue(plan.accepted, plan.decision_reason)
        self.assertGreaterEqual(plan.move_length, .003)
        self.assertLessEqual(plan.move_length, .005)
        p.record_executed_pose(plan.approved_x, plan.approved_y)
        self.assertEqual(p.vertex.state, 'OUTGOING')

    def test_line_mode_retains_centerline_and_does_not_commit(self):
        p = ConnectedPathPlanner(CALIBRATION, {**SETTINGS, 'line_mode': 'Line center'})
        camera = np.asarray([-.15, 0])
        frame = np.full((600,800,3), 255, np.uint8)
        path = np.rint((leg_path()-camera)*PPI + [400,300]).astype(np.int32)
        cv2.polylines(frame, [path], False, (0,0,0), 8)
        p.travel_unit = np.asarray([1.,0.])
        for i in (1,2):
            plan = p.plan(frame, camera_x=camera[0], camera_y=camera[1], belief=belief(camera, i))
            self.assertTrue(plan.accepted, plan.decision_reason)
            self.assertEqual(plan.motion_mode, 'boundary_follow')
        self.assertEqual(p.vertex.state, 'FOLLOW')

    def test_timeline_keeps_transition_fields(self):
        # Use the real schema/writer without instantiating a Tk window.
        from fabscan.camera_calibration import CameraCalibrationDialog
        fields = CameraCalibrationDialog._timeline_fields()
        self.assertEqual(len(fields), len(set(fields)))
        p, camera, polygon = self.committed_setup()
        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        dialog = SimpleNamespace(_timeline_log=lambda event, **kw: writer.writerow({'event':event, **kw}))
        _log_vertex_events(dialog, SimpleNamespace(planner=p), 1, 'test')
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(rows[-1]['vertex_event'], 'COMMITTED')
        self.assertEqual(rows[-1]['vertex_state'], 'APPROACH')
        self.assertEqual(rows[-1]['planner_version'], INTEGRATION_VERSION)
        self.assertAlmostEqual(float(rows[-1]['vertex_x']), 0, delta=.002)
        plan = p.plan(render(camera, polygon), camera_x=camera[0], camera_y=camera[1], belief=belief(camera, 3))
        writer.writerow({'event': 'M6_VERTEX_STATE', **plan.vertex_diagnostics})
        row = list(csv.DictReader(io.StringIO(output.getvalue())))[-1]
        self.assertEqual(row['vertex_association_source'], 'committed_incoming')
        self.assertLessEqual(float(row['vertex_incoming_support_max_px']), 4.0)
        self.assertGreater(float(row['vertex_incoming_support_span']), .08)
        self.assertEqual(row['vertex_corner_connected'], 'True')
        self.assertAlmostEqual(float(row['vertex_corner_radius']), .018)
        self.assertLess(float(row['vertex_tip_distance_px']), 2.0)



def measured_line(planner, frame_camera, point, tangent, confidence=66.318, found=True):
    pixel_error = -planner.machine_to_pixel @ (np.asarray(point)-frame_camera)
    pixel_tangent = -planner.machine_to_pixel @ np.asarray(tangent)
    return SimpleNamespace(found=found, confidence=confidence, pixel_error_x=pixel_error[0],
                           pixel_error_y=pixel_error[1],
                           angle_degrees=np.degrees(np.arctan2(pixel_tangent[1], pixel_tangent[0])))


class EstimatorHandoffTests(unittest.TestCase):
    def build(self, session, camera, frame_camera, frame, line, step):
        return _build_belief(None, session, step_id=step, step_label='estimator integration',
                            line=line, transformed_frame=frame, current_x=camera[0], current_y=camera[1],
                            frame_x=frame_camera[0], frame_y=frame_camera[1])

    def test_real_estimator_through_corner_and_return_to_ordinary_follow(self):
        for sign, degrees, delay in ((1,0,0), (-1,27,.01), (1,90,.01)):
            with self.subTest(sign=sign, degrees=degrees, delay=delay):
                a=np.deg2rad(degrees)
                rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
                incoming=np.array([1.,0.]) @ rot.T
                outgoing=np.array([0.,sign]) @ rot.T
                polygon=np.array([[-1,0],[0,0],[0,sign],[-1,sign]]) @ rot.T
                camera=np.array([-.20,0.]) @ rot.T
                p=ConnectedPathPlanner(CALIBRATION, SETTINGS);p.travel_unit=incoming
                p.record_executed_pose(*camera)
                estimator=PathBeliefEstimator(CALIBRATION, SETTINGS)
                session=RealBeliefSession(estimator,p,CALIBRATION,SETTINGS)
                saw_outgoing=False;resumed=False
                for step in range(1,30):
                    phase=p.vertex.state
                    travel=incoming if not saw_outgoing else outgoing
                    frame_camera=camera-delay*travel
                    frame=render(frame_camera,polygon)
                    # While locked, the generic detector continues reporting an
                    # incoming-biased position/heading, as in the physical run.
                    if p.vertex.active:
                        point=frame_camera+.055*np.array([-incoming[1],incoming[0]])
                        tangent=incoming
                    else:
                        point=frame_camera
                        tangent=outgoing if resumed else incoming
                    line=measured_line(p,frame_camera,point,tangent)
                    b=self.build(session,camera,frame_camera,frame,line,step)
                    plan=p.plan(frame,camera_x=camera[0],camera_y=camera[1],
                                frame_camera_x=frame_camera[0],frame_camera_y=frame_camera[1],belief=b)
                    self.assertTrue(plan.accepted,plan.decision_reason)
                    self.assertGreaterEqual(plan.planner_confidence,45)
                    if phase=='OUTGOING':
                        self.assertEqual(b.measurement_source,'committed_outgoing')
                        np.testing.assert_allclose([b.tangent_x,b.tangent_y],outgoing,atol=.003)
                        self.assertAlmostEqual(b.curvature,0,delta=.001)
                        if not saw_outgoing:
                            self.assertTrue(b.history_reset)
                            self.assertEqual(len(estimator._measurements),1)
                            self.assertTrue(all(k==0 for _,k in estimator._filtered_curvature_history))
                        saw_outgoing=True
                    if resumed:
                        self.assertEqual(b.measurement_source,'detector')
                        self.assertFalse(b.history_reset)
                        self.assertTrue(saw_outgoing)
                        break
                    after=np.array([plan.approved_x,plan.approved_y])
                    session.last_move_x,session.last_move_y=after-camera
                    camera=after
                    _record_executed_belief_pose(session,*camera)
                    resumed=saw_outgoing and p.vertex.state=='FOLLOW'
                else:
                    self.fail('estimator/planner did not return to ordinary follow')
                self.assertGreater(len(estimator.states),len(estimator._measurements))

    def test_committed_measurement_uses_observed_boundary_and_frame_pose(self):
        p,camera,polygon=PlannerTests().committed_setup()
        camera=np.array([-.027,0.]);p.record_executed_pose(*camera)
        frame_camera=camera-[.01,0]
        session=RealBeliefSession(PathBeliefEstimator(CALIBRATION,SETTINGS),p,CALIBRATION,SETTINGS)
        # A 2-pixel actual edge offset must be measured, not silently projected
        # onto the memorized leg. The delayed frame station must also be used.
        frame=render(frame_camera,polygon+[0,.0025])
        line=measured_line(p,frame_camera,frame_camera+[0,.06],[1,0])
        b=self.build(session,camera,frame_camera,frame,line,3)
        self.assertTrue(b.measurement_valid,b.reason)
        self.assertAlmostEqual(b.measurement_y,.0025,delta=.001)
        self.assertAlmostEqual(b.measurement_x,frame_camera[0],delta=.001)
        self.assertEqual(b.measurement_source,'committed_incoming')

    def test_bad_evidence_cannot_fall_back_to_memory_or_blended_detector(self):
        for failure in ('empty','shifted','no_detection','low_confidence'):
            with self.subTest(failure=failure):
                p,camera,polygon=PlannerTests().committed_setup()
                session=RealBeliefSession(PathBeliefEstimator(CALIBRATION,SETTINGS),p,CALIBRATION,SETTINGS)
                frame=render(camera,polygon)
                if failure=='empty':frame[:]=255
                if failure=='shifted':frame=render(camera,polygon+[0,.03])
                line=measured_line(p,camera,camera,[1,0],confidence=20 if failure=='low_confidence' else 85,
                                   found=failure!='no_detection')
                b=self.build(session,camera,camera,frame,line,3)
                self.assertFalse(b.measurement_valid)
                self.assertEqual(b.belief_confidence,0)
                self.assertEqual(len(session.estimator._measurements),0)
                plan=p.plan(frame,camera_x=camera[0],camera_y=camera[1],belief=b)
                self.assertFalse(plan.accepted)
                self.assertEqual(p.vertex.state,'APPROACH')


class RuntimeIntegrationTests(unittest.TestCase):
    def test_physical_step_loop_logs_confirmed_arrival_only_after_success(self):
        from fabscan.camera_calibration import CameraCalibrationDialog

        class Value:
            def __init__(self, value): self.value = value
            def get(self): return self.value
            def set(self, value): self.value = value

        def ignore(*args, **kwargs): pass

        for fail_execution, real_estimator in ((False,False),(True,False),(False,True),(True,True)):
            with self.subTest(fail_execution=fail_execution, real_estimator=real_estimator):
                camera = np.asarray([-.20, 0])
                polygon = np.asarray([[-1,0],[0,0],[0,1],[-1,1]])
                planner = ConnectedPathPlanner(CALIBRATION, SETTINGS)
                planner.travel_unit = np.asarray([1.,0.])
                planner.record_executed_pose(*camera)
                session = RealBeliefSession(PathBeliefEstimator(CALIBRATION, SETTINGS), planner, CALIBRATION, SETTINGS)
                logs = []
                dialog = SimpleNamespace(
                    _timeline_step_counter=0, _active_follow_run_step=0, _follow_stop_requested=False,
                    _timeline_log=lambda event, **kw: logs.append({'event':event, **kw}),
                    _get_follow_direction_preference=lambda: '+X',
                    m6_real_max_move_var=Value(.035), m6_real_min_conf_var=Value(45),
                    m6_real_min_lookahead_var=Value(.045), active_calibration=CALIBRATION,
                    _validate_calibration=lambda value: value, _freshen_follow_frame_if_stale=ignore,
                    linuxcnc_reader=SimpleNamespace(read_status=lambda: SimpleNamespace(connected=True)),
                    _record_position_history=ignore, _status_ok_for_calibration=lambda status: True,
                    _active_position=lambda status: (*camera, 0),
                    _detect_line_for_follow_with_retries=lambda **kw: (
                        measured_line(planner,camera,camera+[0,.06] if planner.vertex.active else camera,
                                      [1,0] if planner.vertex.last_vertex is None else [0,1]),0),
                    get_transformed_frame_bgr=lambda frame: frame,
                    _stabilize_follow_line=lambda line, **kw: line,
                    _get_follow_min_confidence=lambda: 45,
                    _get_follow_deadband=lambda: .003, _get_follow_gain=lambda: .35,
                    _get_follow_max_correct=lambda: .003,
                    _apply_follow_correction_tuning=CameraCalibrationDialog._apply_follow_correction_tuning,
                    _timeline_line_fields=lambda line: {}, cal_status_var=Value(''), update=ignore,
                    _show_current_frame=ignore, _get_follow_feed=lambda: 100,
                    coordinate_mode_label='Machine coordinates', _get_follow_settle_ms=lambda: 60,
                    _wait_and_pump_camera=ignore, follow_capture_point_var=Value(False), trace_capture_callback=None,
                )

                def execute(dialog, **kw):
                    if fail_execution and planner.vertex.active:
                        return False, {}
                    camera[:] = [kw['target_x'], kw['target_y']]
                    return True, {'executor_mode':'mock_xy_executor'}

                belief_context = nullcontext() if real_estimator else patch(
                    'fabscan.real_belief_follow._build_belief', side_effect=lambda *a, **kw: belief(camera, kw['step_id']))
                with belief_context, \
                     patch('fabscan.real_belief_follow._frame_pose', side_effect=lambda *a: (*camera, {})), \
                     patch('fabscan.real_belief_follow._execute_coordinated_xy_step', side_effect=execute):
                    for step in range(25):
                        dialog.current_frame_bgr = render(camera, polygon)
                        dialog.current_frame_sequence = step + 1
                        dialog.current_frame_timestamp = float(step + 1)
                        ok = _m6_step_impl(dialog, session=session, step_label='integration', show_dialogs=False)
                        events = [row.get('vertex_event') for row in logs if row['event']=='M6_VERTEX_TRANSITION']
                        if not ok:
                            self.assertTrue(fail_execution, dialog.cal_status_var.get())
                            self.assertIn('COMMITTED', events)
                            self.assertIn('HOLD', events)
                            self.assertNotIn('VERTEX_REACHED', events)
                            self.assertNotEqual(session.estimator._leg_id, (planner.vertex.vertex_id,'OUTGOING'))
                            break
                        if 'RESUMED' in events:
                            self.assertFalse(fail_execution)
                            self.assertEqual(events, ['CANDIDATE','COMMITTED','VERTEX_REACHED','OUTGOING_LATCHED','RESUMED'])
                            if real_estimator:
                                outgoing = [r for r in logs if r['event']=='M6_BELIEF'
                                            and r.get('belief_measurement_source')=='committed_outgoing']
                                self.assertTrue(outgoing)
                                self.assertTrue(outgoing[0]['belief_history_reset'])
                                self.assertGreaterEqual(float(outgoing[-1]['belief_confidence']),45)
                                dialog.current_frame_bgr=render(camera,polygon)
                                dialog.current_frame_sequence+=1
                                dialog.current_frame_timestamp+=1
                                self.assertTrue(_m6_step_impl(dialog,session=session,step_label='ordinary return',show_dialogs=False),
                                                dialog.cal_status_var.get())
                            break
                    else:
                        self.fail('runtime did not finish expected transition')


if __name__ == '__main__':
    unittest.main()
