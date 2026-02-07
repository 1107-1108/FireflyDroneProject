#define db double

struct _Pid
{
    double kp,ki,kd;

    double integral;
    double lastErr;

    double outLim;
    double intLim;

    _Pid(db _kp,db _ki,db _kd,db _it,db _ltE,db _otL,db _itL):
        kp(_kp),ki(_ki),kd(_kd),integral(_it),lastErr(_ltE),outLim(_otL),intLim(_itL) {}
};

#undef db

static inline double _Lim(double x, double mn, double mx)
{
    return x<mn?mn:(x>mx?mx:x);
}

double Pid_update(_Pid *pid,double target,double measure,double dt)
{
    double dis=target-measure;

    pid->integral+=dis*dt;
    pid->integral=_Lim(pid->integral,-pid->intLim,pid->intLim);

    double der=(dis-pid->lastErr)/dt;
    pid->lastErr=dis;

    double res = pid->kp*dis + pid->ki*pid->integral + pid->kd*der;
    res=_Lim(res,-pid->outLim,pid->outLim);
    return res;
}

void init_pid()
{
    _Pid pid_Roll(0.08,0.002,0.001,0,0,300,200);//
    _Pid pid_Pitch(0.08,0.002,0.001,0,0,300,200);//
    _Pid pid_Yaw(0.12,0.001,0,0,0,300,200);//
}