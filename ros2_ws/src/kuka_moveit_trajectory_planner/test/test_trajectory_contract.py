#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests de la logica pura de kuka_moveit_trajectory_planner.

No requieren ROS 2 en ejecucion ni MoveIt2: solo validan el contrato JSON.

    pytest-3 test/test_trajectory_contract.py
"""

import copy
import math

import pytest

from kuka_moveit_trajectory_planner.trajectory_contract import (
    ContractError,
    build_error_result,
    build_ok_result,
    build_preview_status,
    build_segment,
    build_trajectory_point,
    build_preview_sequence,
    dumps_json,
    extract_trajectory_payload,
    loads_json,
    parse_generation_request,
    parse_preview_request,
    peek_preview_request_id,
    same_pose,
)

JOINTS = ['joint_a1', 'joint_a2', 'joint_a3',
          'joint_a4', 'joint_a5', 'joint_a6']


def make_request(n_points=3, **overrides):
    payload = {
        'schema_version': 1,
        'request_id': 'uuid-test',
        'joint_names': list(JOINTS),
        'points': [
            {'id': f'P{i + 1}', 'joints_deg': [float(i)] * 6}
            for i in range(n_points)
        ],
        'gripper': {
            'initial_state': 'open',
            'events': [{'at_point': 'P2', 'action': 'close'}],
        },
        'planner': {'mode': 'moveit_base', 'execute': False},
    }
    payload.update(overrides)
    return payload


# ── Peticion de generacion ──────────────────────────────────────────────────

def test_segments_are_consecutive_pairs():
    request = parse_generation_request(make_request(4), JOINTS)
    pairs = request.segment_pairs()
    assert request.segment_count == 3
    assert [(i, a.id, b.id) for i, a, b in pairs] == [
        (1, 'P1', 'P2'), (2, 'P2', 'P3'), (3, 'P3', 'P4')]


def test_degrees_are_converted_to_radians():
    payload = make_request(2)
    payload['points'][1]['joints_deg'] = [90.0, -90.0, 180.0, 0.0, 45.0, 0.0]
    request = parse_generation_request(payload, JOINTS)
    assert request.points[1].joints_rad[0] == pytest.approx(math.pi / 2)
    assert request.points[1].joints_rad[1] == pytest.approx(-math.pi / 2)
    assert request.points[1].joints_rad[2] == pytest.approx(math.pi)


def test_gripper_defaults_to_open():
    payload = make_request(2)
    payload.pop('gripper')
    request = parse_generation_request(payload, JOINTS)
    assert request.gripper['initial_state'] == 'open'
    assert request.gripper['events'] == []


def test_gripper_events_are_preserved_verbatim():
    payload = make_request(3)
    payload['gripper']['events'] = [
        {'at_point': 'P2', 'action': 'close', 'width_mm': 30.0}]
    request = parse_generation_request(payload, JOINTS)
    assert request.gripper['events'][0]['width_mm'] == 30.0


def test_unknown_gripper_point_is_a_warning_not_an_error():
    payload = make_request(2)
    payload['gripper']['events'] = [{'at_point': 'P9', 'action': 'close'}]
    request = parse_generation_request(payload, JOINTS)
    assert any('P9' in w for w in request.warnings)


def test_execute_true_is_rejected():
    payload = make_request(2)
    payload['planner']['execute'] = True
    with pytest.raises(ContractError):
        parse_generation_request(payload, JOINTS)


def test_non_base_planner_mode_is_rejected():
    payload = make_request(2)
    payload['planner']['mode'] = 'pilz_lin'
    with pytest.raises(ContractError):
        parse_generation_request(payload, JOINTS)


def test_single_point_is_rejected():
    with pytest.raises(ContractError):
        parse_generation_request(make_request(1), JOINTS)


def test_wrong_joint_count_is_rejected():
    payload = make_request(2)
    payload['points'][0]['joints_deg'] = [0.0, 0.0, 0.0]
    with pytest.raises(ContractError):
        parse_generation_request(payload, JOINTS)


# ── Resultado ───────────────────────────────────────────────────────────────

def test_point_keeps_empty_series_when_moveit_gives_none():
    point = build_trajectory_point(0, 0.0, [0.0] * 6, [], [])
    assert point['velocities_rad_s'] == []
    assert point['accelerations_rad_s2'] == []
    assert point['positions_deg'] == [0.0] * 6


def test_result_summary_counts_every_point():
    request = parse_generation_request(make_request(3), JOINTS)
    segments = []
    for index, from_point, to_point in request.segment_pairs():
        points = [
            build_trajectory_point(i, i * 0.1, [0.0] * 6, [0.0] * 6, [0.0] * 6)
            for i in range(5)
        ]
        segments.append(build_segment(
            index, from_point.id, to_point.id, points, JOINTS))
    result = build_ok_result(request, segments, {'group': 'manipulator'})
    assert result['status'] == 'ok'
    assert result['summary'] == {
        'source_point_count': 3,
        'segment_count': 2,
        'trajectory_point_count': 10,
        'total_duration_sec': pytest.approx(0.8),
    }
    assert [s['segment_id'] for s in result['segments']] == ['T1', 'T2']
    # El resultado debe poder viajar por std_msgs/String.
    assert loads_json(dumps_json(result))['request_id'] == 'uuid-test'


def test_error_result_reports_the_failed_segment():
    result = build_error_result(
        'uuid-test', 'PLANNING_FAILED', failed_segment='T2',
        from_point='P2', to_point='P3', error_code=-1)
    assert result['status'] == 'error'
    assert result['failed_segment'] == 'T2'
    assert result['from_point'] == 'P2'
    assert result['to_point'] == 'P3'
    assert result['moveit_error_code'] == -1


# ── Peticion de preview ─────────────────────────────────────────────────────

def preview_payload():
    return {
        'schema_version': 1,
        'request_id': 'uuid-preview',
        'joint_names': list(JOINTS),
        'segments': [{
            'segment_id': 'T1',
            'from': 'P1',
            'to': 'P2',
            'trajectory_points': [
                {'index': 0, 'time_from_start_sec': 0.0,
                 'positions_rad': [0.0] * 6,
                 'velocities_rad_s': [0.0] * 6,
                 'accelerations_rad_s2': [0.0] * 6},
                {'index': 1, 'time_from_start_sec': 0.5,
                 'positions_rad': [0.1] * 6,
                 'velocities_rad_s': [],
                 'accelerations_rad_s2': []},
            ],
        }],
    }


def test_preview_parses_generator_output():
    request = parse_preview_request(preview_payload(), JOINTS)
    assert len(request.segments) == 1
    assert request.total_point_count == 2
    assert request.total_duration_sec == pytest.approx(0.5)
    assert request.segments[0].points[1].velocities_rad_s == []


def test_preview_accepts_degrees_only_points():
    payload = preview_payload()
    for point in payload['segments'][0]['trajectory_points']:
        point.pop('positions_rad')
        point['positions_deg'] = [90.0] * 6
    request = parse_preview_request(payload, JOINTS)
    assert request.segments[0].points[0].positions_rad[0] == pytest.approx(
        math.pi / 2)


def test_preview_rejects_decreasing_time():
    payload = preview_payload()
    payload['segments'][0]['trajectory_points'][1][
        'time_from_start_sec'] = -1.0
    with pytest.raises(ContractError):
        parse_preview_request(payload, JOINTS)


def test_preview_rejects_missing_segments():
    with pytest.raises(ContractError):
        parse_preview_request({'request_id': 'x', 'segments': []}, JOINTS)


def test_preview_drops_incomplete_series_instead_of_inventing_values():
    payload = preview_payload()
    payload['segments'][0]['trajectory_points'][0]['velocities_rad_s'] = [
        0.0, None, 0.0, 0.0, 0.0, 0.0]
    request = parse_preview_request(payload, JOINTS)
    assert request.segments[0].points[0].velocities_rad_s == []
    assert request.warnings


def test_preview_status_payload():
    status = build_preview_status('uuid-preview', 'playing')
    assert status['status'] == 'playing'
    assert status['request_id'] == 'uuid-preview'


# ── Preview: formato ENVUELTO {"trajectory": {...}} ─────────────────────────

def wrapped_preview_payload():
    """El formato que publica realmente la GUI al releer un archivo guardado."""
    return {
        'schema_version': 1,
        'preview_id': 'preview-123',
        'request_id': 'uuid-guardado',
        'source_file': '/home/gui/secuencias/secuencia_01.json',
        'trajectory': preview_payload(),
    }


def test_extract_trajectory_payload_unwraps_both_contracts():
    direct = preview_payload()
    assert extract_trajectory_payload(direct) is direct

    wrapped = wrapped_preview_payload()
    assert extract_trajectory_payload(wrapped) is wrapped['trajectory']
    assert 'segments' in extract_trajectory_payload(wrapped)


def test_extract_trajectory_payload_rejects_non_object_trajectory():
    with pytest.raises(ContractError):
        extract_trajectory_payload({'trajectory': ['no', 'es', 'objeto']})


def test_preview_parses_wrapped_contract():
    request = parse_preview_request(wrapped_preview_payload(), JOINTS)
    assert len(request.segments) == 1
    assert request.total_point_count == 2
    assert request.segments[0].segment_id == 'T1'
    assert request.joint_names == JOINTS


def test_preview_parses_direct_contract():
    request = parse_preview_request(preview_payload(), JOINTS)
    assert len(request.segments) == 1
    assert request.joint_names == JOINTS
    assert request.preview_id == ''


def test_wrapped_joint_names_come_from_inside_trajectory():
    """joint_names debe leerse del mismo nivel que segments, no de la raiz."""
    payload = wrapped_preview_payload()
    renamed = [f'{name}_real' for name in JOINTS]
    payload['trajectory']['joint_names'] = renamed
    # Ruido a proposito en el nivel raiz: NO debe usarse.
    payload['joint_names'] = ['no', 'usar', 'esto']
    request = parse_preview_request(payload, JOINTS)
    assert request.joint_names == renamed


def test_wrapped_identity_is_preserved():
    request = parse_preview_request(wrapped_preview_payload(), JOINTS)
    assert request.request_id == 'uuid-guardado'
    assert request.preview_id == 'preview-123'


def test_wrapped_request_id_falls_back_to_the_trajectory():
    payload = wrapped_preview_payload()
    payload.pop('request_id')
    payload['trajectory']['request_id'] = 'uuid-interno'
    request = parse_preview_request(payload, JOINTS)
    assert request.request_id == 'uuid-interno'


def test_peek_preview_request_id_looks_at_both_levels():
    assert peek_preview_request_id(wrapped_preview_payload()) == 'uuid-guardado'
    assert peek_preview_request_id(preview_payload()) == 'uuid-preview'
    assert peek_preview_request_id({'trajectory': {'request_id': 'x'}}) == 'x'
    assert peek_preview_request_id({}) == ''


def test_wrapped_without_segments_still_fails_clearly():
    payload = wrapped_preview_payload()
    payload['trajectory'].pop('segments')
    with pytest.raises(ContractError) as excinfo:
        parse_preview_request(payload, JOINTS)
    assert 'trajectory' in str(excinfo.value)


# ── Integracion: archivo guardado -> request de preview -> parse ────────────

SEGMENT_POINT_COUNTS = [16, 16, 15, 16, 15, 15, 15, 15]     # 8 segmentos, 123 puntos


def build_saved_sequence():
    """Reproduce el archivo que guarda el otro entorno: 8 segmentos, 123 puntos."""
    payload = make_request(len(SEGMENT_POINT_COUNTS) + 1)
    payload['request_id'] = 'uuid-guardado'
    request = parse_generation_request(payload, JOINTS)

    segments = []
    for index, from_point, to_point in request.segment_pairs():
        n_points = SEGMENT_POINT_COUNTS[index - 1]
        points = [
            build_trajectory_point(
                i,
                round(i * 0.05, 6),
                [round(index * 0.1 + i * 0.001 + j * 0.01, 6)
                 for j in range(6)],
                [round(i * 0.002 + j * 0.01, 6) for j in range(6)],
                [round(i * 0.003 + j * 0.01, 6) for j in range(6)],
            )
            for i in range(n_points)
        ]
        segments.append(build_segment(
            index, from_point.id, to_point.id, points, JOINTS))

    return build_ok_result(request, segments, {'group': 'manipulator'})


def test_saved_file_to_preview_request_keeps_the_eight_segments(tmp_path):
    """archivo guardado -> construir request de preview -> parse_preview()."""
    # 1. El otro entorno guarda el resultado en disco.
    saved = build_saved_sequence()
    assert saved['summary'] == {
        'source_point_count': 9,
        'segment_count': 8,
        'trajectory_point_count': 123,
        'total_duration_sec': pytest.approx(sum(
            (n - 1) * 0.05 for n in SEGMENT_POINT_COUNTS)),
    }
    path = tmp_path / 'secuencia_01.json'
    path.write_text(dumps_json(saved), encoding='utf-8')

    # 2. La GUI lo relee y construye la peticion de preview (formato envuelto).
    from_disk = loads_json(path.read_text(encoding='utf-8'))
    preview_request_msg = dumps_json({
        'schema_version': 1,
        'preview_id': 'preview-abc',
        'request_id': from_disk['request_id'],
        'source_file': str(path),
        'trajectory': from_disk,
    })

    # 3. El nodo de preview lo recibe por std_msgs/String y lo parsea.
    request = parse_preview_request(loads_json(preview_request_msg), JOINTS)

    assert len(request.segments) == 8
    assert request.total_point_count == 123
    assert [s.segment_id for s in request.segments] == [
        'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8']
    assert [len(s.points) for s in request.segments] == SEGMENT_POINT_COUNTS
    assert request.request_id == 'uuid-guardado'
    assert request.preview_id == 'preview-abc'
    assert request.warnings == []

    # 4. Ningun valor numerico cambia al pasar por el parser.
    for saved_segment, parsed_segment in zip(saved['segments'],
                                             request.segments):
        for saved_point, parsed_point in zip(
                saved_segment['trajectory_points'], parsed_segment.points):
            assert parsed_point.positions_rad == saved_point['positions_rad']
            assert parsed_point.velocities_rad_s == saved_point[
                'velocities_rad_s']
            assert parsed_point.accelerations_rad_s2 == saved_point[
                'accelerations_rad_s2']
            assert parsed_point.time_from_start_sec == saved_point[
                'time_from_start_sec']


def test_saved_file_also_parses_without_the_wrapper(tmp_path):
    """El formato directo anterior debe seguir funcionando igual."""
    saved = build_saved_sequence()
    path = tmp_path / 'secuencia_01.json'
    path.write_text(dumps_json(saved), encoding='utf-8')

    request = parse_preview_request(
        loads_json(path.read_text(encoding='utf-8')), JOINTS)

    assert len(request.segments) == 8
    assert request.total_point_count == 123
    assert request.request_id == 'uuid-guardado'


# ── Secuencia continua de preview (antiparpadeo) ────────────────────────────
#
# Los datos guardados repiten la pose de cada frontera (Ti[-1] == T(i+1)[0]) y
# cada segmento reinicia su reloj en 0.0. Eso NO se toca: la deduplicacion y el
# reloj global existen solo dentro del mecanismo de preview.

PREVIEW_DT = 0.05


def chained_segment_dict(index, n_points, start_value, end_value,
                         dt=PREVIEW_DT):
    """Un segmento cuyo ultimo punto es exactamente end_value en los 6 joints."""
    points = []
    for k in range(n_points):
        frac = k / (n_points - 1) if n_points > 1 else 1.0
        value = round(start_value + (end_value - start_value) * frac, 9)
        points.append({
            'index': k,
            'time_from_start_sec': round(k * dt, 9),
            'positions_rad': [value] * 6,
            'velocities_rad_s': [round(0.01 * k, 9)] * 6,
            'accelerations_rad_s2': [round(0.02 * k, 9)] * 6,
        })
    return {
        'segment_id': f'T{index}',
        'from': f'P{index}',
        'to': f'P{index + 1}',
        'trajectory_points': points,
    }


def chained_preview_request(counts, chained=True):
    """PreviewRequest con fronteras duplicadas (chained=True) o no."""
    segments = []
    for i, n_points in enumerate(counts):
        start_value = round((i + 1) * 0.1, 9)
        end_value = round((i + 2) * 0.1, 9)
        if not chained:
            # Rompe la frontera: el segmento arranca en otra pose.
            start_value = round(start_value + 0.5, 9)
        segments.append(
            chained_segment_dict(i + 1, n_points, start_value, end_value))
    payload = {
        'schema_version': 1,
        'request_id': 'uuid-continuo',
        'joint_names': list(JOINTS),
        'segments': segments,
    }
    return payload, parse_preview_request(payload, JOINTS)


def test_same_pose_respects_the_tolerance():
    assert same_pose([0.0] * 6, [0.0] * 6)
    assert same_pose([0.0] * 6, [1e-9] * 6)
    assert not same_pose([0.0] * 6, [1e-3] * 6)
    assert not same_pose([0.0] * 6, [0.0] * 5)


def test_two_chained_segments_play_the_boundary_pose_once():
    """Requisito 1: segment1[-1] == segment2[0] se reproduce una sola vez."""
    _, request = chained_preview_request([5, 4])
    sequence = build_preview_sequence(request)

    assert sequence.point_count == 5 + 4 - 1
    assert sequence.dropped_boundary_count == 1

    boundary_pose = request.segments[0].points[-1].positions_rad
    repeats = [f for f in sequence.frames
               if f.positions_rad == boundary_pose]
    assert len(repeats) == 1

    # La pose que sobrevive es la del final de T1, y T2 sigue justo despues.
    assert sequence.frames[4].positions_rad == boundary_pose
    assert sequence.frames[5].segment_id == 'T2'
    assert sequence.frames[5].source_point_index == 1


def test_many_chained_segments_drop_only_the_duplicated_boundaries():
    """Requisito 2: 4 segmentos encadenados => 3 fronteras eliminadas."""
    counts = [6, 5, 7, 4]
    _, request = chained_preview_request(counts)
    sequence = build_preview_sequence(request)

    assert sequence.dropped_boundary_count == len(counts) - 1
    assert sequence.point_count == sum(counts) - (len(counts) - 1)
    # Ninguna pose consecutiva se repite en las fronteras.
    starts = [f for f in sequence.frames if f.is_segment_start]
    assert [f.segment_id for f in starts] == ['T1', 'T2', 'T3', 'T4']
    for frame in starts[1:]:
        previous = sequence.frames[sequence.frames.index(frame) - 1]
        assert previous.positions_rad != frame.positions_rad


def test_nothing_is_dropped_when_the_boundary_differs():
    """Requisito 3: si segment1[-1] != segment2[0] no se elimina nada."""
    counts = [5, 4, 6]
    _, request = chained_preview_request(counts, chained=False)
    sequence = build_preview_sequence(request)

    assert sequence.dropped_boundary_count == 0
    assert sequence.point_count == sum(counts)
    for index, segment in enumerate(request.segments):
        frames = [f for f in sequence.frames if f.segment_index == index]
        assert len(frames) == len(segment.points)
        assert frames[0].source_point_index == 0


def test_deduplication_does_not_change_any_value():
    """Requisito 4: posiciones, velocidades y aceleraciones intactas."""
    counts = [6, 5, 7]
    _, request = chained_preview_request(counts)
    sequence = build_preview_sequence(request)

    for frame in sequence.frames:
        source = request.segments[frame.segment_index].points[
            frame.source_point_index]
        assert frame.positions_rad == source.positions_rad
        assert frame.velocities_rad_s == source.velocities_rad_s
        assert frame.accelerations_rad_s2 == source.accelerations_rad_s2


def test_original_segments_are_untouched_by_the_preview_build():
    """Requisito 5: la trayectoria original conserva sus segmentos y puntos."""
    counts = [6, 5, 7, 4]
    payload, request = chained_preview_request(counts)
    before = copy.deepcopy(payload['segments'])
    snapshot = [[list(p.positions_rad) for p in s.points]
                for s in request.segments]

    sequence = build_preview_sequence(request)

    assert len(request.segments) == len(counts)
    assert [len(s.points) for s in request.segments] == counts
    assert request.total_point_count == sum(counts)
    assert payload['segments'] == before
    assert [[list(p.positions_rad) for p in s.points]
            for s in request.segments] == snapshot
    # Y las listas de la secuencia son copias, no las mismas referencias.
    sequence.frames[0].positions_rad[0] = 99.0
    assert request.segments[0].points[0].positions_rad[0] != 99.0


def test_preview_clock_is_monotonic_and_never_restarts():
    """Requisito 6: reloj global monotono que no vuelve a cero."""
    counts = [6, 5, 7, 4]
    _, request = chained_preview_request(counts)
    sequence = build_preview_sequence(request)

    times = sequence.times
    assert times[0] == 0.0
    assert all(b >= a for a, b in zip(times, times[1:]))
    # Ninguna pose posterior a la primera vuelve al instante 0.
    assert all(t > 0.0 for t in times[1:])
    # En las fronteras el reloj avanza, no se reinicia.
    for frame in sequence.frames:
        if frame.is_segment_start and frame.segment_index > 0:
            position = sequence.frames.index(frame)
            assert frame.time_from_start_sec > sequence.frames[
                position - 1].time_from_start_sec
    # Con fronteras duplicadas el paso original se conserva: rampa uniforme.
    expected = [round(i * PREVIEW_DT, 9) for i in range(len(times))]
    assert times == pytest.approx(expected)


def test_real_case_eight_segments_123_points_gives_116_poses():
    """Requisito 7: 123 puntos - 7 fronteras duplicadas = 116 poses."""
    counts = SEGMENT_POINT_COUNTS                      # [16,16,15,16,15,15,15,15]
    assert len(counts) == 8 and sum(counts) == 123

    payload, request = chained_preview_request(counts)
    wrapped = {'preview_id': 'prev-1', 'request_id': 'uuid-continuo',
               'trajectory': payload}
    request = parse_preview_request(loads_json(dumps_json(wrapped)), JOINTS)
    sequence = build_preview_sequence(request)

    assert len(request.segments) == 8
    assert request.total_point_count == 123
    assert sequence.dropped_boundary_count == 7
    assert sequence.point_count == 116
    assert [f.segment_id for f in sequence.frames if f.is_segment_start] == [
        'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8']
    assert sequence.duration_sec == pytest.approx(115 * PREVIEW_DT)


def test_no_reset_or_empty_pose_is_inserted_between_segments():
    """Requisito 8: ni poses vacias, ni repeticiones, ni saltos al inicio."""
    counts = [6, 5, 7, 4]
    _, request = chained_preview_request(counts)
    sequence = build_preview_sequence(request)

    first_pose = sequence.frames[0].positions_rad
    for index, frame in enumerate(sequence.frames):
        # Ninguna pose vacia o de longitud distinta (un "reset" visual).
        assert len(frame.positions_rad) == len(JOINTS)
        assert frame.positions_rad
        if index == 0:
            continue
        previous = sequence.frames[index - 1]
        # Nunca se reinyecta la pose inicial a mitad de la reproduccion.
        assert frame.positions_rad != first_pose
        # Nunca se repite la pose anterior (eso es lo que se veia parpadear).
        assert frame.positions_rad != previous.positions_rad
        # El indice de origen avanza dentro del segmento, sin volver atras.
        if frame.segment_index == previous.segment_index:
            assert frame.source_point_index == previous.source_point_index + 1

    # El numero de poses es exactamente el esperado: no se inserta ninguna.
    assert sequence.point_count == sum(counts) - (len(counts) - 1)


def test_single_point_segment_is_never_emptied():
    """Un segmento degenerado de 1 punto conserva su pose (y su status)."""
    payload, request = chained_preview_request([4, 1])
    sequence = build_preview_sequence(request)
    assert [f.segment_id for f in sequence.frames if f.is_segment_start] == [
        'T1', 'T2']


def test_gap_is_only_added_when_the_boundary_was_not_duplicated():
    """El hueco opcional no se aplica a fronteras ya deduplicadas."""
    _, chained = chained_preview_request([5, 4])
    with_gap = build_preview_sequence(chained, inter_segment_gap_sec=1.0)
    without_gap = build_preview_sequence(chained)
    assert with_gap.times == without_gap.times

    _, broken = chained_preview_request([5, 4], chained=False)
    gapped = build_preview_sequence(broken, inter_segment_gap_sec=1.0)
    boundary = [f for f in gapped.frames if f.is_segment_start][1]
    index = gapped.frames.index(boundary)
    assert boundary.time_from_start_sec == pytest.approx(
        gapped.frames[index - 1].time_from_start_sec + 1.0)
