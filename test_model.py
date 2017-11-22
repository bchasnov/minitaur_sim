#!/usr/bin/env python3
"""
Code to load mujoco model and simulate control input.

Author:
    Benjamin Chasnov (bchasnov@uw.edu)

Example usage:
    python run_model.py xmls/snapper_1d.xml --loop 100 --demo 3

"""
import argparse
from mujoco_py import load_model_from_path, MjSim, MjViewer
import math

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model_file', type=str)
    parser.add_argument('--max_time', type=int, default=1000)
    parser.add_argument('--loop', type=int, default=3)
    parser.add_argument('--demo', type=int, default=0)

    args = parser.parse_args()
    model_file = args.model_file

    model = load_model_from_path(model_file)
    sim = MjSim(model)
    viewer = MjViewer(sim)

    t = 0
    loop = 0

    while True:
        t += 1
        sim.step()
        viewer.render()

        mode = args.demo

        if mode == 1:
            mini_hop(sim, t)
        elif mode == 2:
            tall_hop(sim, t)
        elif mode == 3:
            sine_move(sim, t, args.max_time)
        elif mode == 4:
            wiggle_move(sim, t, args.max_time)
        else:
            sim.data.ctrl[:] = 0

        # break conditions
        if t > args.max_time:
            loop += 1
            t = 0
        if loop > args.loop:
            break


def mini_hop(sim, t):
    # step functions ___-----___
    if t < 500:
        control(sim, 0, 0)
    elif t < 700:
        control(sim, 0.5, -0.5)
    elif t < 800:
        control(sim, 0, 0)

def tall_hop(sim, t):
    if t < 500:
        control(sim, 0, 0)
    elif t < 700:
        control(sim, 1, -1)
    elif t < 800:
        control(sim, 0, 0)

def sine_move(sim, t, max_time):
    period = 2*math.pi/max_time
    amplitude = 1
    offset = 0

    x = amplitude * math.sin(t * period) + offset

    control(sim, x, -1*x)

def wiggle_move(sim, t, max_time):
    period = 2*math.pi/max_time
    phase = math.pi/2
    amplitude = 1
    offset = 0

    x = amplitude * math.sin(t * period) + offset
    xp = amplitude * math.sin(t * period + phase) + offset

    control(sim, x, -1*xp)


def control(sim, u1, u3):
    # position and velocity of leg1
    sim.data.ctrl[0] = u1
    sim.data.ctrl[1] = 0.02

    # position and velocity of leg2
    sim.data.ctrl[2] = u3
    sim.data.ctrl[3] = 0.02

    try:
        sim.data.ctrl[4] = u1
        sim.data.ctrl[5] = 0.02

        sim.data.ctrl[6] = u3
        sim.data.ctrl[7] = 0.02
    except:
        pass

    try:
        sim.data.ctrl[8] = u1
        sim.data.ctrl[9] = 0.02

        sim.data.ctrl[10] = u3
        sim.data.ctrl[11] = 0.02

        sim.data.ctrl[12] = u1
        sim.data.ctrl[13] = 0.02

        sim.data.ctrl[14] = u3
        sim.data.ctrl[15] = 0.02
    except:
        pass


if __name__ == '__main__':
    main()
