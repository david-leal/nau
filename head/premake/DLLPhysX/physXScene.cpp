#include "physXScene.h"

void getMatFromPhysXTransform(physx::PxTransform transform, float* matrix) {
	physx::PxMat44 mat = physx::PxMat44(transform);
	for (int i = 0; i < 16; ++i) matrix[i] = *(mat.front() + i);
}