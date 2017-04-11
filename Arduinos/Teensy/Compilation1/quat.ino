
/**
   Calculate product of two quaternions
*/
void quatProd(float* q1, float* q2, float* qprod)
{

  qprod[0] = q2[0] * q1[0] - q2[1] * q1[1] - q2[2] * q1[2] - q2[3] * q1[3];
  qprod[1] = q2[0] * q1[1] + q2[1] * q1[0] - q2[2] * q1[3] + q2[3] * q1[2];
  qprod[2] = q2[0] * q1[2] + q2[1] * q1[3] + q2[2] * q1[0] - q2[3] * q1[1];
  qprod[3] = q2[0] * q1[3] - q2[1] * q1[2] + q2[2] * q1[1] + q2[3] * q1[0];

}
/**
   Calculate conjugate of a quaternion
*/
void quatConj(float* qin, float* qout)
{
  qout[0] = qin[0];
  qout[1] = -qin[1];
  qout[2] = -qin[2];
  qout[3] = -qin[3];
}
/**
   Calculate norm of a quaternion
*/
float quatNorm(float* q)
{
  return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}
