package main

import (
	"math"
	"time"
)

func bodyBodyInteraction(
	fx *float32, fy *float32, fz *float32,
	x0 float32, y0 float32, z0 float32,
	x1 float32, y1 float32, z1 float32, mass1 float32,
	softeningSquared float32) {

	dx := x1 - x0
	dy := y1 - y0
	dz := z1 - z0

	distSqr := dx*dx + dy*dy + dz*dz
	distSqr += softeningSquared

	// BUG: This single line is responsible for utterly destroying the
	// performance. There are many things wrong with it:
	//
	//   * It requires float64. This requires a type conversion *and* it's
	//     a very slow calculation.
	//   * gccgo doesn't recognize the 1.0 / sqrt() pattern as something that
	//     could be lowered to reciprocal square root instructions.
	//   * It breaks the auto-vectorization surrounding it.
	//
	// In C we can just write '1.0f / sqrtf(distSqr)' and GCC makes none of the
	// mistakes above, and even recognizes that it can use a packed vector
	// reciprocal square root instruction in the auto-vectorized version.
	//
	invDist := 1.0 / float32(math.Sqrt(float64(distSqr)))

	invDistCube := invDist * invDist * invDist
	s := mass1 * invDistCube

	*fx = dx * s
	*fy = dy * s
	*fz = dz * s
}

func ComputeGravitation_AOS(force []float32, posMass []float32, softeningSquared float32, N int) float32 {

	start := time.Now()
	for i := 0; i < N; i++ {
		var acx, acy, acz float32 = 0.0, 0.0, 0.0
		myX := posMass[i*4+0]
		myY := posMass[i*4+1]
		myZ := posMass[i*4+2]

		for j := 0; j < N; j++ {

			var fx, fy, fz float32
			bodyX := posMass[j*4+0]
			bodyY := posMass[j*4+1]
			bodyZ := posMass[j*4+2]
			bodyMass := posMass[j*4+3]

			bodyBodyInteraction(
				&fx, &fy, &fz,
				myX, myY, myZ,
				bodyX, bodyY, bodyZ, bodyMass,
				softeningSquared )

			acx += fx
			acy += fy
			acz += fz
		}

		force[3*i+0] = acx
		force[3*i+1] = acy
		force[3*i+2] = acz
	}
	end := time.Now()

	return float32(end.Sub(start).Nanoseconds()) * 1e-6
}

/* vim: set ts=4 sts=4 sw=4 noet: */
