/*
 * QuadController.hpp
 *
 *  Created on: Sep 9, 2014
 *      Author: walmis
 */

#ifndef QUADCONTROLLER_HPP_
#define QUADCONTROLLER_HPP_

#include <xpcc/architecture.hpp>

#define M_FRONT_LEFT 1
#define M_FRONT_RIGHT 0
#define M_REAR_LEFT 2
#define M_REAR_RIGHT 3

using namespace xpcc;
using namespace xpcc::lpc17;
class Blinker : TickerTask {
	PeriodicTimer<> t;
public:
	Blinker() : t(800) {}

	enum {
		STATE_WAITING,
		STATE_SIGNAL,
		STATE_ARMED
	};

	void handleTick() override {
		if(t.isExpired()) {
			ledRed::toggle();
		}
	}

public:
	void setState(int state) {
		switch(state) {
		case STATE_WAITING:
			t.restart(800);
			break;
		case STATE_SIGNAL:
			t.restart(300);
			break;
		case STATE_ARMED:
			t.restart(100);
			break;
		}
	}
};

Blinker blinker;


class QuadController : public SensorProcessor {

public:

	QuadController() :
		rollController(0, 0, 0, 0, 1.0f),
		pitchController(0, 0, 0, 0, 1.0f),
		qTarget(1.0, 0, 0, 0)
	{

		Pinsel::setFunc(2,2,1);
		Pinsel::setFunc(2,3,1);
		Pinsel::setFunc(2,4,1);
		Pinsel::setFunc(2,5,1);

		PWM::initTimer(10);

		const int freq = 100;
		prescale = SystemCoreClock / 10 / freq;

		PWM::matchUpdate(0, prescale);

		PWM::configureMatch(0, PWM::MatchFlags::RESET_ON_MATCH);

		PWM::channelEnable(3);
		PWM::channelEnable(4);
		PWM::channelEnable(5);
		PWM::channelEnable(6);

		PWM::matchUpdate(3, 0);
		PWM::matchUpdate(4, 0);
		PWM::matchUpdate(5, 0);
		PWM::matchUpdate(6, 0);

		PWM::enable();

	}

	struct DataPkt {
		struct {float w;float x;float y;float z;} rotationQ;
		struct {float w;float x;float y;float z;} targetQ;
		struct {float x;float y;float z;} gyro;
		struct {float x;float y;float z;} acc;
		struct {float x;float y;float z;} mag;
		struct {float x;float y;float z;} dynamicAcc;
		struct {float x;float y;float z;} velocity;
		struct {uint8_t a;uint8_t b;uint8_t c;uint8_t d;} pwm;
		float height;
		uint8_t throttle;
	} __attribute__((packed));

	void fillPacket(DataPkt &packet) {
		packet.rotationQ.w = qRotation.w;
		packet.rotationQ.x = qRotation.x;
		packet.rotationQ.y = qRotation.y;
		packet.rotationQ.z = qRotation.z;

		packet.targetQ.w = qTarget.w;
		packet.targetQ.x = qTarget.x;
		packet.targetQ.y = qTarget.y;
		packet.targetQ.z = qTarget.z;

		packet.gyro.x = vGyro.x;
		packet.gyro.y = vGyro.y;
		packet.gyro.z = vGyro.z;

		packet.acc.x = vAcc.x;
		packet.acc.y = vAcc.y;
		packet.acc.z = vAcc.z;

		packet.mag.x = vMag.x;
		packet.mag.y = vMag.y;
		packet.mag.z = vMag.z;

		packet.dynamicAcc.x = dynamicAcc.x;
		packet.dynamicAcc.y = dynamicAcc.y;
		packet.dynamicAcc.z = dynamicAcc.z;

		packet.velocity.x = velocity.x;
		packet.velocity.y = velocity.y;
		packet.velocity.z = velocity.z;

		packet.pwm.a = motorSpeeds[0]*255.0;
		packet.pwm.b = motorSpeeds[1]*255.0;
		packet.pwm.c = motorSpeeds[2]*255.0;
		packet.pwm.d = motorSpeeds[3]*255.0;

		packet.height = height;
		packet.throttle = (throttle + throttleComp)*255.0;
	}

	void handleInit() override {

		if(eeprom.isValidToken()) {

			float pid[3];
			float hpid[4];

			XPCC_LOG_DEBUG .printf("loading PID params\n");
			eeprom.eeRead(EEData::PIDParams, pid);
			setPID(pid[0], pid[1], pid[2], false);

			eeprom.eeRead(EEData::ratePIDParams, pid);
			setRatePID(pid[0], pid[1], pid[2], false);

			eeprom.eeRead(EEData::heightPIDparams, hpid);
			setHeightPID(hpid[0], hpid[1], hpid[2], hpid[3], false);

			eeprom.eeRead(EEData::yawGain, yawGain);

		} else {

			const float rpid[3] = {0.1070, 0.0015, 0};
			const float pid[3]  = {1.4700, 0.0700, 0};
			const float hpid[4] = {0, 0, 0.25, 0.2};

			setPID(pid[0], pid[1], pid[2]);
			setRatePID(rpid[0], rpid[1], rpid[2]);
			setHeightPID(hpid[0], hpid[1], hpid[2], hpid[3]);

			eeprom.eeWrite(EEData::yawGain, yawGain);
		}

		this->SensorProcessor::handleInit();
	}

	bool isArmed() {
		return pidEnable;
	}

	void handleTick() override {
		static PeriodicTimer<> updateTimer(10);

		if(updateTimer.isExpired()) {
			static PeriodicTimer<> tPrint(200);
			static Vector3f v;

//			if(pwmInputs.isActive()) {
//
//				if(tPrint.isExpired()) {
//					XPCC_LOG_DEBUG .printf("y:%.2f p:%.2f r:%.2f t:%.2f %d\n", pwmInputs.getYaw(),
//							pwmInputs.getPitch(), pwmInputs.getRoll(),
//							pwmInputs.getThrottle(), pwmInputs.getAux());
//
//					//XPCC_LOG_DEBUG .printf("h %.3f\n", height);
//					//XPCC_LOG_DEBUG << "Target " << qTarget << endl;
//				}
//
//				if(pwmInputs.getAux()) {
//
//					if(!isArmed()) {
//						if(pwmInputs.getThrottle() < 0.05) {
//							arm(true);
//						}
//					}
//					static Vector3f v;
//
//					v[0] -= pwmInputs.getYaw() * (45 * M_PI/180.0 * 0.010);
//					v[1] = pwmInputs.getPitch() * (15 * M_PI/180.0);
//					v[2] = pwmInputs.getRoll() * (15 * M_PI/180.0);
//
//					qTarget = Quaternion<float>(v[0], v[1], v[2]);
//
//					throttle = pwmInputs.getThrottle();
//
//				} else {
//					if(isArmed() && pwmInputs.getThrottle() < 0.05) {
//						arm(false);
//					}
//				}
//
//			}
//			else {
//				XPCC_LOG_DEBUG .printf("!");
//
//				qTarget = Quaternion<float>(v[0], 0, 0);
//
//			}


		}
		this->SensorProcessor::handleTick();
	}

	void setPID(float Kp, float Ki, float Kd, bool store = true) {
		Pid<float, 1>::Parameter p(Kp, Ki, Kd, 5.0, 20.0);

		rollController.setParameter(p);
		pitchController.setParameter(p);
		yawController.setParameter(p);

		if(store) {
			float pid[3] = {Kp,Ki,Kd};
			eeprom.eeWrite(EEData::PIDParams, pid);
		}
	}

	void setHeightPID(float Kp, float Ki, float Kd, float maxOutput = 0.2, bool store = true) {
		Pid<float, 1>::Parameter p(Kp, Ki, Kd, 5.0, maxOutput);

		heightController.setParameter(p);

		if(store) {
			float pid[4] = {Kp, Ki, Kd, maxOutput};
			eeprom.eeWrite(EEData::heightPIDparams, pid);
		}

	}

	void setRatePID(float Kp, float Ki, float Kd, bool store = true) {
		Pid<float, 1>::Parameter p(Kp, Ki, Kd, 1.0, 1.0);

		rollRateController.setParameter(p);
		pitchRateController.setParameter(p);
		yawRateController.setParameter(p);

		if(store) {
			float pid[3] = {Kp,Ki,Kd};
			eeprom.eeWrite(EEData::ratePIDParams, pid);
		}
	}

	void arm(bool armed) {
		XPCC_LOG_DEBUG .printf("ARM %d\n", armed);
		rollController.reset();
		yawController.reset();
		pitchController.reset();
		rollRateController.reset();
		pitchRateController.reset();
		yawRateController.reset();
		heightController.reset();

		throttle = 0;
		throttleComp = 0;

		if(armed) {
			pidEnable = true;
			blinker.setState(Blinker::STATE_ARMED);
		} else {
			pidEnable = false;
			blinker.setState(Blinker::STATE_SIGNAL);

			stopMotors = true;
		}
	}



	void updateRatePID(float dt) {
		Vector3f error = vGyro - targetVAngular;

		rollRateController.update(error.y, dt);
		pitchRateController.update(error.x, dt);
		yawRateController.update(error.z, dt);

		float pitchSet = pitchRateController.getValue();
		float rollSet = rollRateController.getValue();
		float yawSet = yawRateController.getValue() * yawGain;

		//XPCC_LOG_DEBUG .printf("err sum %.4f\n", rollController.getErrorSum());

		//stream.printf("pid %.4f\n", rollSet);

		const float minSpeed = 0.05f;

		float t = throttle + throttleComp;

		float motors[4] = {t,t,t,t};

		motors[M_FRONT_RIGHT] += yawSet;
		motors[M_REAR_LEFT]   += yawSet;
		motors[M_FRONT_LEFT]  -= yawSet;
		motors[M_REAR_RIGHT]  -= yawSet;

		motors[M_FRONT_LEFT] -= rollSet;
		motors[M_REAR_LEFT] -= rollSet;

		motors[M_FRONT_RIGHT] += rollSet;
		motors[M_REAR_RIGHT] += rollSet;

		motors[M_FRONT_LEFT] -= pitchSet;
		motors[M_FRONT_RIGHT] -= pitchSet;

		motors[M_REAR_LEFT] += pitchSet;
		motors[M_REAR_RIGHT] += pitchSet;

		for(int i =0; i < 4; i++) {
			if(motors[i] > 1.0f) motors[i] = 1.0f;
			else if(motors[i] < minSpeed) motors[i] = minSpeed;
		}

		setMotorOutput(motors);
	}

	void onSensorsCalibrated() override {
		float spd[] = {0,0,0,0};
		setMotorOutput(spd);
	}

	void updatePID(float dt) {
		float z0 = qRotation.w*-qTarget.z + qRotation.z*qTarget.w + qRotation.x*-qTarget.y - qRotation.y*-qTarget.x;
		float w0 = qRotation.w*qTarget.w - qRotation.x*-qTarget.x - qRotation.y*-qTarget.y - qRotation.z*-qTarget.z;

		float eyaw = 2 * atanf(z0 / w0);

		Vector3f err = getGravity(qRotation).cross(getGravity(qTarget));

		err.x = asinf(err.x);
		err.y = asinf(err.y);

		//XPCC_LOG_DEBUG << "err " << err << "eyaw: " << eyaw << endl;

		//ex = 2*atanf(qError.x/qError.w);
		//ey = 2*atanf(qError.y/qError.w);
		//XPCC_LOG_DEBUG .printf("x:%.4f y:%.4f ez:%.4f y:%.4f\n", ex, ey, ez, eyaw);

		//XPCC_LOG_DEBUG .printf ("%.3f %.3f %.3f y:%.3f\n", m[2][0], m[2][1], m[2][2], yaw);

//		Matrix3f m;
//		qError.to3x3Matrix(&m);
//		XPCC_LOG_DEBUG << "matrix\n" << m << endl;

		//Vector3f verr = Vector3f(qError.x, qError.y, qError.z);

		//Vector3f gravity = getGravity(qRotation).rotated(qError);

		//Vector3f err = getYawPitchRoll(qRotation, gravity);

		//XPCC_LOG_DEBUG << "err " << err << endl;

		//take yaw error from quaternion
		//err.x = qError.z;

		pitchController.update(-err.x, dt);
		rollController.update(-err.y, dt);
		yawController.update(-eyaw, dt);

		Vector3f v(pitchController.getValue(),
				rollController.getValue(),
				yawController.getValue());

		targetVAngular = v;

	}

	void onHeightUpdated(float dt) {
		if(isArmed()) {
			heightController.update(-height, dt);

			throttleComp = heightController.getValue();

			//XPCC_LOG_DEBUG .printf("h pid %.3f %.3f\n", height, heightController.getValue());
		}
	}

	void onSensorsUpdated(float dt) override {
		if(isArmed()) {
			updateRatePID(dt);
			updatePID(dt);
		} else {
			if(stopMotors) {
				const float mot[4] = {0,0,0,0};
				setMotorOutput(mot);
				stopMotors = false;
			}
		}
	}

/*
	 ---       ---
	| 1 |     | 0 |
	 ---   ^   ---
		\  ^ /
		 \  /
		 /  \
		/    \
	 ---      ---
	| 2 |    | 3 |
	 ---      ---
*/
	void setMotorOutput(const float speeds[4]) {
		int min = prescale / 10; //1ms pulsewidth = 0% motor output
		int max = prescale / 5; //2ms pulsewidth = 100% motor output
		max /= 2;

		memcpy(motorSpeeds, speeds, sizeof(float)*4);

		//XPCC_LOG_DEBUG .printf("%.2f %.2f %.2f %.2f\n", speeds[0], speeds[1], speeds[2], speeds[3]);

		auto m = PWM::multiMatchUpdate();
		m.set(3, min + max * speeds[0]);
		m.set(4, min + max * speeds[1]);
		m.set(5, min + max * speeds[2]);
		m.set(6, min + max * speeds[3]);
		m.commit(PWM::UpdateType::PWM_MATCH_UPDATE_NEXT_RST);
	}

	//state variables
	volatile bool pidEnable;
	volatile bool stopMotors;

	//absolute throttle value that is used by the controllers
	float throttle;

	//throttle compensation
	float throttleComp;

	DataPkt packet;

	//desired quadrotor orientation
	Quaternion<float> qTarget;

	//desired angular rates
	Vector3f targetVAngular;

	//PID controllers
	Pid<float> rollRateController;
	Pid<float> pitchRateController;
	Pid<float> yawRateController;
	Pid<float> rollController;
	Pid<float> pitchController;
	Pid<float> yawController;
	Pid<float> heightController;

	float yawGain = 1.7;

	//Radio controller pwm inputs
	//ControllerInputs pwmInputs;

private:
	uint32_t prescale;

	float motorSpeeds[4];
};


#endif /* QUADCONTROLLER_HPP_ */
