
static void calculateHandlePosition(FSCtrl_Vars_t   *pFSCtrl)
{
    int idx;
    for (idx = 0; idx < MTR_NUM; idx ++)
    {
        pFSCtrl->bigarmPos[idx].x = 0;
        pFSCtrl->bigarmPos[idx].y = __cospuf32(pFSCtrl->armAngles[idx]) * pFSCtrl->bigarm_length;
        pFSCtrl->bigarmPos[idx].z = __sinpuf32(pFSCtrl->armAngles[idx]) * pFSCtrl->bigarm_length + pFSCtrl->motor_offset;
    }

    // Rotate 120 CCW around axis Y to calculate arm joint position of MTR_3 on the right side
    pFSCtrl->bigarmPos[MTR_3].x = pFSCtrl->bigarmPos[MTR_3].z * SQRT3byTwo;
    pFSCtrl->bigarmPos[MTR_3].z = -pFSCtrl->bigarmPos[MTR_3].z/2;

    // Rotate 120 CW around axis Y to calculate arm joint position of MTR_1 on the left side
    pFSCtrl->bigarmPos[MTR_1].x = -pFSCtrl->bigarmPos[MTR_1].z * SQRT3byTwo;
    pFSCtrl->bigarmPos[MTR_1].z = -pFSCtrl->bigarmPos[MTR_1].z/2;

    calc_circular_cone_vertex(pFSCtrl->bigarmPos, pFSCtrl->handle_offset, pFSCtrl->forearm_length, &pFSCtrl->handlePos);

}

/**
 * calculate the position of load cell base center
 * pAx, pAy, pAz-p,
 * pBx-sqrt(3)/2*p, pBy, pBz+1/2*p,
 * pCx+sqrt(3)/2*p, pCy, pCz+1/2*p
 */

static void calc_circular_cone_vertex(vector3d_t *points, float32_t d, float32_t length, vector3d_t *vertex)
{
    float32_t x1 = points[1].x,
          x2 = points[2].x - SQRT3byTwo * d,
          x3 = points[0].x + SQRT3byTwo * d;
    float32_t y1 = points[1].y,
          y2 = points[2].y,
          y3 = points[0].y;
    float32_t z1 = points[1].z - d,
          z2 = points[2].z + d/2,
          z3 = points[0].z + d/2;

    float32_t a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2),
           b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2),
           c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2),
           d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

    float32_t a2 = 2 * (x2 - x1),
           b2 = 2 * (y2 - y1),
           c2 = 2 * (z2 - z1),
           d2 = x1*x1 + y1*y1 + z1*z1 - x2*x2 - y2*y2 - z2*z2;

    float32_t a3 = 2 * (x3 - x1),
           b3 = 2 * (y3 - y1),
           c3 = 2 * (z3 - z1),
           d3 = x1*x1 + y1*y1 + z1*z1 - x3*x3 - y3*y3 - z3*z3;

    float32_t cx = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1)
            /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    float32_t cy =  (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1)
            /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    float32_t cz = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1)
            /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);

    float32_t l2 = length*length - (x1-cx) * (x1-cx) - (y1-cy) * (y1-cy) - (z1-cz) * (z1-cz);
    float32_t t2 = l2/(a1*a1+b1*b1+c1*c1);
    float32_t t = -__sqrtf(t2);

    vertex->x = cx + a1*t;
    vertex->y = cy + b1*t;
    vertex->z = cz + c1*t;
}
