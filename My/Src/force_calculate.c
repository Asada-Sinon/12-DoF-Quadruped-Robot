#include "force_calculate.h"
#include "matrix.h"
#include "dog.h"

// 施工中

static float I3[3][3] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float Pcom[3] = {0, 0, 0};
float P_body[4][3] = {0};

// 目标力与力矩，暂时直接给出，后续由速度pd计算给出
float F[3] = {0};
float T[3] = {0};

float F_foot[4][3] = {0}; // 计算得到的足端力

float g[3] = {0, 0, -9.81};
float w[12] = {10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4};
float u[12] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
float alpha = 0.001;
float beta = 0.1;
float fricRatio = 0.4;
float s[6] = {20, 20, 50, 450, 450, 450};

float A_f32[6][12] = {0};
float AT_f32[12][6] = {0};
float S_f32[6][6] = {0};
float ATS_f32[12][6] = {0};
float ATSA_f32[12][12] = {0};
float W_f32[12][12] = {0};
float U_f32[12][12] = {0};
float G_f32[12][12] = {0};
float b_f32[6][1] = {0};
float bT_f32[1][6] = {0};
float bTS_f32[1][6] = {0};
float bTSA_f32[1][12] = {0};
float Fprev_f32[12][1] = {0};
float FprevT_f32[1][12] = {0};
float g0T_f32[1][12] = {0};
float fricMat[5][3] = {0};


arm_matrix_instance_f32 G;
arm_matrix_instance_f32 A;
arm_matrix_instance_f32 AT;
arm_matrix_instance_f32 S;
arm_matrix_instance_f32 ATS;
arm_matrix_instance_f32 ATSA;
arm_matrix_instance_f32 W;
arm_matrix_instance_f32 U;
arm_matrix_instance_f32 b;
arm_matrix_instance_f32 bT;
arm_matrix_instance_f32 bTS;
arm_matrix_instance_f32 bTSA;
arm_matrix_instance_f32 Fprev;
arm_matrix_instance_f32 FprevT;
arm_matrix_instance_f32 g0T;

void force_init()
{

    // 初始化S，W，U对角矩阵
    for(int i = 0; i < 6; i++)
    {   
        S_f32[i][i] = s[i];
    }
    for(int i = 0; i < 12; i++)
    {   
        W_f32[i][i] = w[i];
        U_f32[i][i] = u[i];
    }

    // 初始化A矩阵
    //[I3       I3       I3       I3      ]
    //[[P0-Pc]x [P1-Pc]x [P2-Pc]x [P3-Pc]x] 6x12
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            A_f32[j][i * 3 + j] = 1;
        }
    }
    for(int i = 0; i < 4; i++)
    {
        int j = 3;
        leg_get_current_foot_pos_body(i, P_body[i]);

        A_f32[j][i * 3 + 1] = -(P_body[i][2] - Pcom[2]);
        A_f32[j][i * 3 + 2] = -(P_body[i][1] - Pcom[1]);
        A_f32[j + 1][i * 3] = (P_body[i][2] - Pcom[2]);
        A_f32[j + 1][i * 3 + 2] = -(P_body[i][0] - Pcom[0]);
        A_f32[j + 2][i * 3] = -(P_body[i][1] - Pcom[1]);
        A_f32[j + 2][i * 3 + 1] = (P_body[i][0] - Pcom[0]);
    }

    // 初始化摩擦锥约束矩阵
    //[1 ,  0, _fricRatio]
    //[-1,  0, _fricRatio]
    //[0 ,  1, _fricRatio]
    //[0 , -1, _fricRatio]
    //[0 ,  0,          1]
    fricMat[0][0] = 1;
    fricMat[0][1] = 0;
    fricMat[0][2] = fricRatio;
    fricMat[1][0] = -1;
    fricMat[1][1] = 0;
    fricMat[1][2] = fricRatio;
    fricMat[2][0] = 0;
    fricMat[2][1] = 1;
    fricMat[2][2] = fricRatio;
    fricMat[3][0] = 0;
    fricMat[3][1] = -1;
    fricMat[3][2] = fricRatio;
    fricMat[4][0] = 0;
    fricMat[4][1] = 0;
    fricMat[4][2] = 1;

}

void force_calculate()
{
    // 计算优化目标J = 1/2*f^T*(A^TSA + alpha*W + beta*U)*f + (-bd^T*S*A - beta*Fprev^T*U)*f
    // 计算G矩阵
    arm_mat_init_f32(&G, 12, 12, (float32_t*)G_f32);
    arm_mat_init_f32(&A, 6, 12, (float32_t*)A_f32);
    arm_mat_init_f32(&AT, 12, 6, (float32_t*)AT_f32);
    arm_mat_init_f32(&S, 6, 6, (float32_t*)S_f32);
    arm_mat_init_f32(&ATS, 12, 6, (float32_t*)ATS_f32);
    arm_mat_init_f32(&ATSA, 12, 12, (float32_t*)ATSA_f32);
    arm_mat_init_f32(&W, 12, 12, (float32_t*)W_f32);
    arm_mat_init_f32(&U, 12, 12, (float32_t*)U_f32);

    // 更新A矩阵
    for(int i = 0; i < 4; i++)
    {
        int j = 3;
        leg_get_current_foot_pos_body(i, P_body[i]);

        A_f32[j][i * 3 + 1] = -(P_body[i][2] - Pcom[2]);
        A_f32[j][i * 3 + 2] = -(P_body[i][1] - Pcom[1]);
        A_f32[j + 1][i * 3] = (P_body[i][2] - Pcom[2]);
        A_f32[j + 1][i * 3 + 2] = -(P_body[i][0] - Pcom[0]);
        A_f32[j + 2][i * 3] = -(P_body[i][1] - Pcom[1]);
        A_f32[j + 2][i * 3 + 1] = (P_body[i][0] - Pcom[0]);
    }

    // 计算G = A^T*S*A + alpha*W + beta*U
    arm_mat_trans_f32(&A, &AT);
    arm_mat_mult_f32(&AT, &S, &ATS);
    arm_mat_mult_f32(&ATS, &A, &G);
    // W,U是对角阵，如果计算时间过长，可以考虑直接加到G矩阵上
    // arm_mat_add_f32(&G, &W, &G);
    // arm_mat_add_f32(&G, &U, &G);
    for(int i = 0; i < 12; i++)
    {
        G_f32[i][i] += alpha * W_f32[i][i] + beta * U_f32[i][i];
    }
    
    // 计算g0T = -b^T*S*A - beta*Fprev^T*U
    arm_mat_init_f32(&b, 6, 1, (float32_t*)b_f32);
    arm_mat_init_f32(&bT, 1, 6, (float32_t*)bT_f32);
    arm_mat_init_f32(&bTS, 1, 6, (float32_t*)bTS_f32);
    arm_mat_init_f32(&bTSA, 1, 12, (float32_t*)bTSA_f32);
    arm_mat_init_f32(&Fprev, 12, 1, (float32_t*)Fprev_f32);
    arm_mat_init_f32(&FprevT, 1, 12, (float32_t*)FprevT_f32);
    arm_mat_init_f32(&g0T, 1, 12, (float32_t*)g0T_f32);

    arm_mat_trans_f32(&b, &bT);
    arm_mat_mult_f32(&bT, &S, &bTS);
    arm_mat_mult_f32(&bTS, &A, &bTSA);
    arm_mat_scale_f32(&bTSA, -1, &g0T);

    for(int i = 0; i < 12; i++)
    {
        FprevT_f32[0][i] = -1 *Fprev_f32[i][0] * beta * U_f32[i][i];
    }

    arm_mat_add_f32(&g0T, &FprevT, &g0T);
}


// BalanceCtrl::BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta)
//             : _mass(mass), _Ib(Ib), _S(S), _alpha(alpha), _beta(beta){
//     _Fprev.setZero();
//     _g << 0, 0, -9.81;
//     _fricRatio = 0.3;
//     _fricMat <<  1,  0, _fricRatio,
//                 -1,  0, _fricRatio,
//                  0,  1, _fricRatio,
//                  0, -1, _fricRatio,
//                  0,  0, 1;
// }

// BalanceCtrl::BalanceCtrl(QuadrupedRobot *robModel){
//     Vec6 s;
//     Vec12 w, u;

//     _mass = robModel->getRobMass();
//     _pcb = robModel->getPcb();
//     _Ib = robModel->getRobInertial();
//     _g << 0, 0, -9.81;

//     w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
//     u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
//     _alpha = 0.001;
//     _beta  = 0.1;
//     _fricRatio = 0.4;

//     s << 20, 20, 50, 450, 450, 450; 

//     _S = s.asDiagonal();
//     _W = w.asDiagonal();
//     _U = u.asDiagonal();
    
//     _Fprev.setZero();
//     _fricMat <<  1,  0, _fricRatio,
//                 -1,  0, _fricRatio,
//                  0,  1, _fricRatio,
//                  0, -1, _fricRatio,
//                  0,  0, 1;
// }

// Vec34 BalanceCtrl::calF(Vec3 ddPcd, Vec3 dWbd, RotMat rotM, Vec34 feetPos2B, VecInt4 contact){
//     calMatrixA(feetPos2B, rotM, contact);
//     calVectorBd(ddPcd, dWbd, rotM);
//     calConstraints(contact);

//     _G = _A.transpose()*_S*_A + _alpha*_W + _beta*_U;
//     _g0T = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;

//     solveQP();

//     _Fprev = _F;
//     return vec12ToVec34(_F);
// }

// void BalanceCtrl::calMatrixA(Vec34 feetPos2B, RotMat rotM, VecInt4 contact){
//     for(int i(0); i < 4; ++i){
//         _A.block(0, 3*i, 3, 3) = I3;
//         _A.block(3, 3*i, 3, 3) = skew(feetPos2B.col(i) - rotM*_pcb);
//     }
// }

// void BalanceCtrl::calVectorBd(Vec3 ddPcd, Vec3 dWbd, RotMat rotM){
//     _bd.head(3) = _mass * (ddPcd - _g);
//     _bd.tail(3) = (rotM * _Ib * rotM.transpose()) * dWbd;
// }

// void BalanceCtrl::calConstraints(VecInt4 contact){
//     int contactLegNum = 0;
//     for(int i(0); i<4; ++i){
//         if(contact(i) == 1){
//             contactLegNum += 1;
//         }
//     }

//     _CI.resize(5*contactLegNum, 12);
//     _ci0.resize(5*contactLegNum);
//     _CE.resize(3*(4 - contactLegNum), 12);
//     _ce0.resize(3*(4 - contactLegNum));

//     _CI.setZero();
//     _ci0.setZero();
//     _CE.setZero();
//     _ce0.setZero();

//     int ceID = 0;
//     int ciID = 0;
//     for(int i(0); i<4; ++i){
//         if(contact(i) == 1){
//             _CI.block(5*ciID, 3*i, 5, 3) = _fricMat;
//             ++ciID;
//         }else{
//             _CE.block(3*ceID, 3*i, 3, 3) = I3;
//             ++ceID;
//         }
//     }
// }

float CI_f32[20][12] = {0};  // 最大20行（4条腿 * 5行/腿）
float CI0_f32[20] = {0};     // 最大20个元素
float CE_f32[12][12] = {0};  // 最大12行（4条腿 * 3行/腿）
float CE0_f32[12] = {0};     // 最大12个元素

void calConstraints()
{
    // 计算接触腿的数量
    int contactLegNum = 0;
    int contact[4] = {1};
    
    for(int i = 0; i < 4; i++)
    {
        contact[i] = leg_get_contact_state(i);
        if(contact[i] == 1)
        {
            contactLegNum++;
        }
    }

    // 初始化约束矩阵尺寸
    int CI_rows = 5 * contactLegNum;
    int CE_rows = 3 * (4 - contactLegNum);
    int CI_cols = 12;
    int CE_cols = 12;

    // 填充约束矩阵
    memset(CI_f32, 0, sizeof(CI_f32));
    memset(CE_f32, 0, sizeof(CE_f32));

    int ceID = 0;
    int ciID = 0;

    
    for(int i = 0; i < 4; i++)
    {
        if(contact[i] == 1)
        {
            // 添加摩擦锥约束
            for(int row = 0; row < 5; row++)
            {
                for(int col = 0; col < 3; col++)
                {
                    CI_f32[5*ciID + row][3*i + col] = fricMat[row][col];
                }
            }
            ciID++;
        }
        else
        {
            // 添加零力约束
            for(int j = 0; j < 3; j++)
            {
                CE_f32[3*ceID + j][3*i + j] = 1;
            }
            ceID++;
        }
    }
}

// void BalanceCtrl::solveQP(){
//     int n = _F.size();
//     int m = _ce0.size();
//     int p = _ci0.size();

//     G.resize(n, n);
//     CE.resize(n, m);
//     CI.resize(n, p);
//     g0.resize(n);
//     ce0.resize(m);
//     ci0.resize(p);
//     x.resize(n);

//     for (int i = 0; i < n; ++i) {
//         for (int j = 0; j < n; ++j) {
//             G[i][j] = _G(i, j);
//         }
//     }

//     for (int i = 0; i < n; ++i) {
//         for (int j = 0; j < m; ++j) {
//             CE[i][j] = (_CE.transpose())(i, j);
//         }
//     }

//     for (int i = 0; i < n; ++i) {
//         for (int j = 0; j < p; ++j) {
//             CI[i][j] = (_CI.transpose())(i, j);
//         }
//     }

//     for (int i = 0; i < n; ++i) {
//         g0[i] = _g0T[i];
//     }

//     for (int i = 0; i < m; ++i) {
//         ce0[i] = _ce0[i];
//     }

//     for (int i = 0; i < p; ++i) {
//         ci0[i] = _ci0[i];
//     }

//     double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

//     for (int i = 0; i < n; ++i) {
//         _F[i] = x[i];
//     }
// }
