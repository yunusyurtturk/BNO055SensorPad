#pragma once

ref class RelativeMovement
{
private:
	float x, y, z;
	float x_last, y_last, z_last;
public:
	RelativeMovement() {
		Init();
	}
	void Init()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;

		x_last = 0.0f;
		y_last = 0.0f;
		z_last = 0.0f;
	}

	void Reset()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	void OneStep(float x_acc, float y_acc, float z_acc)
	{
		x += (x_acc - x_last) * (0.1);
		y += (y_acc - y_last) * (0.1);
		z += (z_acc - z_last) * (0.1);

		x_last = x_acc;
		y_last = y_acc;
		z_last = z_acc;
	}

	void GetVector(float *p_dest) {
		p_dest[0] = x;
		p_dest[1] = y;
		p_dest[2] = z;
	}

};