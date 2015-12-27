package main

import (
	"time"
)

func ComputeGravitation_SOA(force [][]float32, pos [][]float32, mass []float32, softeningSquared float32, N int) float32 {

	start := time.Now()
	for i := 0; i < N; i++ {
		var acx, acy, acz float32 = 0.0, 0.0, 0.0
		myX := pos[0][i]
		myY := pos[1][i]
		myZ := pos[2][i]

		for j := 0; j < N; j++ {

			var fx, fy, fz float32
			bodyX := pos[0][j]
			bodyY := pos[1][j]
			bodyZ := pos[2][j]
			bodyMass := mass[j]

			bodyBodyInteraction(
				&fx, &fy, &fz,
				myX, myY, myZ,
				bodyX, bodyY, bodyZ, bodyMass,
				softeningSquared )

			acx += fx
			acy += fy
			acz += fz
		}

		force[0][i] = acx
		force[1][i] = acy
		force[2][i] = acz
	}
	end := time.Now()

	return float32(end.Sub(start).Nanoseconds()) * 1e-6
}

/* vim: set ts=4 sts=4 sw=4 noet: */
