package main

import (
	"fmt"
	"math/rand"
)

var g_hostAOS_PosMass []float32
var g_hostAOS_VelInvMass []float32
var g_hostAOS_Force []float32

var g_hostAOS_Force_Golden []float32

var g_hostSOA_Pos [][]float32
var g_hostSOA_Force [][]float32
var g_hostSOA_Mass []float32
var g_hostSOA_InvMass []float32

var g_Algorithms = [...]string {
	"CPU_AOS",
	"CPU_SOA",
}

var g_softening float32 = 0.1
var g_damping float32 = 0.995
var g_dt float32 = 0.016

var g_Algorithm int
var g_CrossCheck bool
var g_CycleAfter int
var g_MaxIterations int
var g_N int

func randomVector(v []float32) {
	var lenSqr float32
	for {
		v[0] = rand.Float32()
		v[1] = rand.Float32()
		v[2] = rand.Float32()
		lenSqr = v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
		if lenSqr <= 1.0 {
			break
		}
	}
}

func randomUnitBodies(pos []float32, vel []float32, N int) {
	for i := 0; i < N; i++ {
		base := 4*i
		randomVector( pos[base:base+3] );
		randomVector( vel[base:base+3] );
		pos[4*i+3] = 1.0 // unit mass
		vel[4*i+3] = 1.0
	}
}

func allocArrays() {
	g_hostSOA_Pos = make([][]float32, 3)
	g_hostSOA_Force = make([][]float32, 3)
	for i := 0; i < 3; i++ {
		g_hostSOA_Pos[i] = make([]float32, g_N)
		g_hostSOA_Force[i] = make([] float32, g_N)
	}
	g_hostSOA_Mass = make([] float32, g_N)
	g_hostSOA_InvMass = make([]float32, g_N)

	g_hostAOS_PosMass = make([]float32, 4 * g_N)
	g_hostAOS_Force = make([]float32, 3 * g_N)
	g_hostAOS_Force_Golden = make([]float32, 3 * g_N)
	g_hostAOS_VelInvMass = make([]float32, 4 * g_N)
}

func integrateGravitation_AOS(ppos []float32, pvel []float32, pforce []float32, dt float32, damping float32, N int) {
	for i := 0; i < N; i++ {
		var index int = 4*i
		var indexForce int = 3*i

		var pos, vel, force [3]float32
		pos[0] = ppos[index+0]
		pos[1] = ppos[index+1]
		pos[2] = ppos[index+2]
		invMass := pvel[index+3]

		vel[0] = pvel[index+0]
		vel[1] = pvel[index+1]
		vel[2] = pvel[index+2]

		force[0] = pforce[indexForce+0]
		force[1] = pforce[indexForce+1]
		force[2] = pforce[indexForce+2]

		// acceleration = force / mass
		// new velocity = old velocity + acceleration * deltaTime
		vel[0] += (force[0] * invMass) * dt
		vel[1] += (force[1] * invMass) * dt
		vel[2] += (force[2] * invMass) * dt

		vel[0] *= damping
		vel[1] *= damping
		vel[2] *= damping

		// new position = old position + velocity * deltaTime
		pos[0] += vel[0] * dt
		pos[1] += vel[1] * dt
		pos[2] += vel[2] * dt

		ppos[index+0] = pos[0]
		ppos[index+1] = pos[1]
		ppos[index+2] = pos[2]

		pvel[index+0] = vel[0]
		pvel[index+1] = vel[1]
		pvel[index+2] = vel[2]
	}
}

func computeGravitation(ms *float32, err *float32, algorithm string, crossCheck bool) {

	// AOS -> SOA data structures in case we are measuring SOA performance
	for i := 0; i < g_N; i++ {
		g_hostSOA_Pos[0][i]  = g_hostAOS_PosMass[4*i+0]
		g_hostSOA_Pos[1][i]  = g_hostAOS_PosMass[4*i+1]
		g_hostSOA_Pos[2][i]  = g_hostAOS_PosMass[4*i+2]
		g_hostSOA_Mass[i]    = g_hostAOS_PosMass[4*i+3]
		g_hostSOA_InvMass[i] = 1.0 / g_hostSOA_Mass[i]
	}

	// TODO: Implement cross-check here.

	for i := 0; i < g_N; i++ {
		for j := 0; j < 3; j++ {
			g_hostAOS_Force[3*i+j] = 0.0
			g_hostSOA_Force[j][i] = 0.0
		}
	}

	var bSOA bool = false

	switch algorithm {
	case "CPU_AOS":
		*ms = ComputeGravitation_AOS(
			g_hostAOS_Force,
			g_hostAOS_PosMass,
			g_softening * g_softening,
			g_N);
	case "CPU_SOA":
		*ms = ComputeGravitation_SOA(
			g_hostSOA_Force,
			g_hostSOA_Pos,
			g_hostSOA_Mass,
			g_softening * g_softening,
			g_N)
		bSOA = true
	}

	if bSOA {
		for i := 0; i < g_N; i++ {
			g_hostAOS_Force[3*i+0] = g_hostSOA_Force[0][i]
			g_hostAOS_Force[3*i+1] = g_hostSOA_Force[1][i]
			g_hostAOS_Force[3*i+2] = g_hostSOA_Force[2][i]
		}
	}

	*err = 0.0
	if crossCheck {
		// TODO
	}

	integrateGravitation_AOS(
		g_hostAOS_PosMass,
		g_hostAOS_VelInvMass,
		g_hostAOS_Force,
		g_dt,
		g_damping,
		g_N)
}

func main() {
	fmt.Println("N-body prototype")

	// TODO: Option parsing
	g_N = 1024
	g_CrossCheck = false
	g_Algorithm = 1

	allocArrays()
	randomUnitBodies(g_hostAOS_PosMass, g_hostAOS_VelInvMass, g_N)
	for i:= 0; i < g_N; i++ {
		g_hostSOA_Mass[i] = g_hostAOS_PosMass[4*i+3]
		g_hostSOA_InvMass[i] = 1.0 / g_hostSOA_Mass[i]
	}

	running := true
	steps, iterations := 0, 0

	for {
		if !running {
			break
		}

		var ms float32
		var err float32

		computeGravitation(&ms, &err, g_Algorithms[g_Algorithm], g_CrossCheck)

		var interactionsPerSecond float32
		var flops float32

		interactionsPerSecond = float32(g_N) * float32(g_N) * 1000.0 / ms
		flops = interactionsPerSecond * float32(3 + 6 + 4 + 1 + 6) * 1e-3;

		if interactionsPerSecond > 1e9 {
			fmt.Printf("\r%13s: %8.2f ms = %8.3fx10^9 interactions/s (%9.2f GFLOPS)",
				g_Algorithms[g_Algorithm],
				ms,
				interactionsPerSecond / 1e9,
				flops * 1e-6);
		} else {
			fmt.Printf("\r%13s: %8.2f ms = %8.3fx10^6 interactions/s (%9.2f GFLOPS)",
				g_Algorithms[g_Algorithm],
				ms,
				interactionsPerSecond / 1e6,
				flops * 1e-6);
		}
		if g_CrossCheck {
			fmt.Printf(" (Rel. error: %E)\n", err);
		} else {
			fmt.Printf("\n");
		}

		steps++
		if g_CycleAfter != 0 && steps % g_CycleAfter == 0 {
		} else if g_CycleAfter == 0 {
			iterations++
		}

		if g_MaxIterations != 0 && iterations > g_MaxIterations {
			running = false
		}
	}
}

/* vim: set ts=4 sts=4 sw=4 noet: */
