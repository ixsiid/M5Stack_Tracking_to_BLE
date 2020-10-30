#include <M5Stack.h>
#include <stdio.h>

// #include "MadgwickAHRS.h"

// using namespace AHRS;

#define TICK_TYPE int64_t
#define TICK_FUNC esp_timer_get_time
#define PULSE_PERIOD_THRESHOLD 2000

struct BaseStationSignal_t {
	volatile TICK_TYPE rise_a;
	volatile uint rise_offset_b_from_a;
	volatile int pulse_width_a;
	volatile int pulse_width_b;
	volatile uint pulse_offset_signal_from_a;
};

struct _BaseStationSignal_t {
	TICK_TYPE rise_a;
	uint rise_offset_b_from_a;
	int pulse_width_a;
	int pulse_width_b;
	uint pulse_offset_signal_from_a;
};

BaseStationSignal_t pulse[8] = {0};
volatile int pulse_index;
volatile TICK_TYPE rise;
volatile uint rise_offset;

void begin(uint8_t pin) {
	pulse_index = 0;
	rise		  = 0;
	rise_offset = 0;

	pinModex();
	attachInterrupt(BaseStation_interrupt);
}

void IRAM_ATTR BaseStation_interrupt() {
	if (isHigh() && !rise) {
		rise_offset = rise;
		rise		  = TICK_FUNC();
		rise_offset -= rise;
	} else if (rise) {
		int width = (TICK_FUNC() - rise);
		if (width < 56) {
			// 短パルス受信
			BaseStationSignal_t* t = pulse[(pulse_index - 1) & 0b111];
			if (!t->pulse_offset_signal_from_a) t->pulse_offset_signal_from_a = rise_offset;
		} else if (width < 350) {
			// 長パルス
			BaseStationSignal_t* t = pulse[pulse_index];
			if (rise_offset > PULSE_PERIOD_THRESHOLD) {
				// パルス間隔が長いため、Sync信号初期化
				t->rise_a		  = rise;
				t->pulse_width_a = width;
			} else {
				// 2台目のベースステーション
				t->rise_offset_b_from_a		= rise_offset;
				t->pulse_width_b			= width;
				t->pulse_offset_signal_from_a = 0;
				pulse_index				= (pulse_index + 1) & 0b111;
			}
		}
	}
}

_BaseStationSignal_t _pulse[8];

int sum_width_a(int index) {
	int r = 0;
	for (int i = 0; i < 4; i++) {
		r += _pulse[index].pulse_width_a;
		index = (index + 1) & 0b111;
	}
	for (int i = 0; i < 4; i++) {
		r -= _pulse[index].pulse_width_a;
		index = (index + 1) & 0b111;
	}

	return r;
}

void CalculatePosition() {
	memcpy(pulse, _pulse, sizeof(BaseStationSignal_t) * 8);

	int a = sum_width_a(0);
	int b = sum_width_a(1);

	int p;
	if (a > b) {
		for (p = 7; p > 4; p--) {
			b = sum_width_a(p);
			if (a > b) {
				p = (p + 1) & 0b111;
				break;
			}
			a = b;
		}
	} else {
		for (p = 2; p < 5; p++) {
			a = sum_width_a(p);
			if (b > a) {
				p = p - 1;
				break;
			}
			b = a;
		}
	}

	BaseStationSignal_t *as[4];
	BaseStationSignal_t *temp;
	for(int i=0; i<4; i++) {
		as[i] = &_pulse[(p + i) & 0b111];
	}

	// 8個ブロックを仮定したほうがよさそう

	// 4の並び替えをハードコーディング
	if (as[0]->pulse_width_a > as[1]->pulse_width_a) {
		if (as[2]->pulse_width_a > as[3]->pulse_width_a) {
			if (as[0]->pulse_width_a > as[2]->pulse_width_a) {
				// as[0] = as[0]
				if (as[1]->pulse_width_a > as[3]->pulse_width_a) {
					// as[3] = as[3]
					if (as[1]->pulse_width_a > as[2]->pulse_width_a) {
						// as[1] = as[1]
						// as[2] = as[2]
					} else {
						temp = as[1];
						as[1] = as[2];
						as[2] = temp;
					}
				} else {
					temp = as[1];
					as[1] = as[2]; //
					as[2] = as[3]; //
					as[3] = temp; //
				}
			} else {
				temp = as[0];
				as[0] = as[2]; //
				if (as[1]->pulse_width_a > as[3]->pulse_width_a) {
					as[2] = as[1]; //
					as[1] = temp; //
				} else {
					as[2] = temp;
					temp = as[3];
					as[3] = as[1]; //
					if (as[2]->pulse_width_a > temp->pulse_width_a) {
						as[1] = as[2]; //
						as[2] = temp; //
					} else {
						as[1] = temp; //
						// as[2] = as[2]
					}
				}
			}
		} else {
			/// 0 > 1
			/// 2 < 3
			if (as[0]->pulse_width_a > as[3]->pulse_width_a) {
				/// 0 > 1
				/// V
				/// 3 > 2

				// as[0] = as[0]
				if (as[1]->pulse_width_a > as[2]->pulse_width_a) {
					temp = as[3];
					as[3] = as[2];
					if (as[1]->pulse_width_a > temp->pulse_width_a) {
						as[2] = temp; //
					} else {
						as[2] = as[1];
						as[1] = temp;
					}
				} else {
					temp = as[3];
					as[3] = as[1];
					as[1] = temp;
					// as[2] = as[2]
				}
			} else {
				/// 0 > 1
				/// A
				/// 3 > 2

				temp = as[0];
				as[0] = as[3]; //
				if (as[1]->pulse_width_a > as[2]->pulse_width_a) {
					as[3] = as[2];
					as[2] = as[1];
					as[1] = temp;
				} else {
					as[3] = as[1];
					if (as[2]->pulse_width_a > temp->pulse_width_a) {
						as[1] = as[2];
						as[2] = temp;
					} else {
						as[1] = temp;
					}
				}
			}
		}
	} else {
		// 0 < 1
		if (as[2]->pulse_width_a > as[3]->pulse_width_a) {
			/// 0 < 1
			/// 2 > 3
			if (as[1]->pulse_width_a > as[2]->pulse_width_a) {
				temp = as[0];
				as[0] = as[1];
				/// 2 > 3  temp
				if (as[2]->pulse_width_a > temp->pulse_width_a) {
					as[1] = as[2];
					if (temp->pulse_width_a > as[3]->pulse_width_a) {
						as[2] = temp;
					} else {
						as[2] = as[3];
						as[3] = temp;
					}
				} else {
					as[1] = temp;
				}
			} else {
				/// 0 < 1
				///     A
				///     2 > 3, 
				temp = as[0];
				as[0] = as[2]; //
				if (as[3]->pulse_width_a > temp->pulse_width_a) {
					as[2] = as[3];
					as[3] = temp;
					if (as[1]->pulse_width_a > as[2]->pulse_width_a) {
						// as[1] = as[1]
						// as[2] = as[2]
					} else {
						temp = as[1];
						as[1] = as[2];
						as[2] = temp;
					}
				} else {
					// as[1] = as[1]
					as[2] = temp;
					// as[3] = as[3]
				}
			}
			/// ここまで出来た
		} else {
			/// 0 < 1
			/// 2 < 3
			if (as[0]->pulse_width_a > as[3]->pulse_width_a) {
				/// 0 > 1
				/// V
				/// 3 > 2

				// as[0] = as[0]
				if (as[1]->pulse_width_a > as[2]->pulse_width_a) {
					temp = as[3];
					as[3] = as[2];
					if (as[1]->pulse_width_a > temp->pulse_width_a) {
						as[2] = temp; //
					} else {
						as[2] = as[1];
						as[1] = temp;
					}
				} else {
					temp = as[3];
					as[3] = as[1];
					as[1] = temp;
					// as[2] = as[2]
				}
			} else {
				/// 0 > 1
				/// A
				/// 3 > 2

				temp = as[0];
				as[0] = as[3]; //
				if (as[1]->pulse_width_a > as[2]->pulse_width_a) {
					as[3] = as[2];
					as[2] = as[1];
					as[1] = temp;
				} else {
					as[3] = as[1];
					if (as[2]->pulse_width_a > temp->pulse_width_a) {
						as[1] = as[2];
						as[2] = temp;
					} else {
						as[1] = temp;
					}
				}
			}
		}
	}
}

int main() {
	printf("Hello, world");

	//	IAHRS * ahrs = new MadgwickAHRS(0.03f);
	//	printf(" [%x]", ahrs);
}
