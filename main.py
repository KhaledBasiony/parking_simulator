"""
Simulator script for simulating parking algorithms
"""

import math
import ctypes

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle, Circle
import matplotlib

matplotlib.use("QtAgg")


class Simulator:
    """
    Conatins all required data for simulation
    """

    libpath: ctypes.CDLL = ...

    # Figure related fields
    ax: matplotlib.axes.Axes = ...
    fig: matplotlib.figure.Figure = ...
    car: Rectangle = ...
    axle_c: Circle = ...
    path_line: matplotlib.lines.Line2D = ...
    scale: float = ...

    # Model related fields
    # car length
    l_c: float = ...

    # axle-to-axle length
    l_a: float = ...

    # trunk length
    l_t: float = ...

    # car width
    w_c: float = ...

    # parking slot length
    l_s: float = ...

    # parking slot width/depth
    w_s: float = ...

    # side distance from side car
    d_s: float = ...

    # forward distance from side car
    d_f: float = ...

    # Motion planning related fields
    param_a: float = ...
    param_b: float = ...
    x_range: list[int] = ...

    def __init__(
        self,
        scale: float = 10,
        radius: float = 80,
        d_f: float = -1,
        d_s: float = 20,
        l_c: float = 40,
        l_a: float = 30,
        l_t: float = 5,
        w_c: float = 20,
        l_s: float = 60,
        w_s: float = 22,
    ):
        self.fig, self.ax = plt.subplots()
        self.scale = scale
        self.d_f = d_f * scale
        self.d_s = d_s * scale
        self.l_c = l_c * scale
        self.l_a = l_a * scale
        self.l_t = l_t * scale
        self.w_c = w_c * scale
        self.l_s = l_s * scale
        self.w_s = w_s * scale

        self.param_a = radius * scale
        self.x_range = [0, radius * 2]

        self.libpath = ctypes.CDLL("./libpath.so")
        self.libpath.PaB_Path.restype = ctypes.c_float
        
        self.libpath.Set_Param_A(ctypes.c_float(self.param_a))
        self.libpath.Set_Param_B(
            ctypes.c_float((self.w_c / 2 + self.d_s + self.w_s / 2) / 2)
        )

    def path(self, x: float) -> float:
        """Responsible for finding a point on the path"""

        return self.libpath.PaB_Path(ctypes.c_float(x))

    def _base_plot(self):
        # clear at the beginning of animation
        self.ax.clear()

        # ultrasonic range
        us_range = 400 * self.scale

        # axes config
        self.ax.set_xlim([-us_range / 3.75, us_range / 3.75])
        self.ax.set_ylim([-us_range / 3.75, us_range / 3.75])

        xticks = self.ax.get_xticks()
        self.ax.set_xticks(
            xticks,
            labels=list(map(lambda x: str(x / self.scale), xticks)),
        )

        yticks = self.ax.get_yticks()
        self.ax.set_yticks(
            yticks,
            labels=list(map(lambda x: str(x / self.scale), yticks)),
        )

        self.ax.grid(which="both", axis="x")

        # side car
        side_car_width = 20
        side_car_length = 50
        side_car = Rectangle(
            ((self.w_c / 2 + self.d_s), -self.d_f),
            side_car_width * self.scale,
            side_car_length * self.scale,
            color="red",
        )
        self.ax.add_patch(side_car)

        # side car
        side_car = Rectangle(
            ((self.w_c / 2 + self.d_s), -self.d_f - self.l_s),
            side_car_width * self.scale,
            -side_car_length * self.scale,
            color="red",
        )
        self.ax.add_patch(side_car)

        # motion path
        xs = np.linspace(
            self.x_range[0] * self.scale,
            self.x_range[1] * self.scale,
            self.x_range[1] * 2 * self.scale,
        )
        self.path_line = self.ax.plot(
            xs,
            [self.path(x) for x in xs],
        )

        # car
        car = Rectangle(
            (-self.w_c / 2, -self.l_t),
            self.w_c,
            self.l_c,
            color="orange",
        )
        self.car = self.ax.add_patch(car)

        # car rear axle center point
        axle_center = Circle((0, 0), 2 * self.scale, color="black")
        self.axle_c = self.ax.add_patch(axle_center)

    def _update(self, frame):
        new_x = frame - self.w_c / 2
        new_y = self.path(frame) - self.l_t
        angle = math.atan2(new_y - self.car.get_y(), new_x - self.car.get_x())

        self.car.set_xy((new_x, new_y))
        axle = (new_x + self.w_c / 2, new_y + self.l_t)
        self.axle_c.set_center(axle)

        self.car.rotation_point = axle
        self.car.set_angle(angle * 180 / math.pi + 90)

    def main(self, repeat=True):
        """Main function for running the simulator."""

        ani = animation.FuncAnimation(
            fig=self.fig,
            func=self._update,
            init_func=self._base_plot,
            frames=self.x_range[1] * self.scale,
            interval=10,
            repeat_delay=2000,
            repeat=repeat,
        )
        plt.show()


if __name__ == "__main__":
    Simulator(
        radius=24,
        d_s=22,
        scale=5,
    ).main()
