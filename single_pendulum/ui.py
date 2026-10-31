"""
ui.py
=====
Модуль графічних компонентів Pygame для одинарного інвертованого маятника:
адаптивні кнопки, повзунки LQR, осцилограф кута та координати, індикатор прогресу MPC.
Повністю підтримує динамічне масштабування (DPI-aware) під роздільну здатність екрана.
"""

from typing import Callable, List, Optional, Tuple

import numpy as np
import pygame


class Button:
    """Інтерактивна кнопка з підтримкою hover-ефекту, адаптивного шрифту та callback."""

    def __init__(
        self,
        x: int,
        y: int,
        w: int,
        h: int,
        text: str,
        color: Tuple[int, int, int],
        hover_color: Tuple[int, int, int],
        callback: Callable[[], None],
        border_radius: int = 6,
        on_press: Optional[Callable[[], None]] = None,
        on_release: Optional[Callable[[], None]] = None,
    ):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.callback = callback
        self.on_press = on_press
        self.on_release = on_release
        self.border_radius = border_radius
        self.hovered: bool = False
        self.pressed: bool = False

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.pressed = True
                if self.on_press:
                    self.on_press()
                self.callback()
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self.pressed:
                self.pressed = False
                if self.on_release:
                    self.on_release()
                return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        col = self.hover_color if (self.hovered or self.pressed) else self.color
        pygame.draw.rect(surface, col, self.rect, border_radius=self.border_radius)
        border_col = (220, 230, 245) if self.hovered else (90, 105, 130)
        pygame.draw.rect(
            surface, border_col, self.rect, width=1, border_radius=self.border_radius
        )

        txt_surf = font.render(self.text, True, (255, 255, 255))
        txt_rect = txt_surf.get_rect(center=self.rect.center)
        surface.blit(txt_surf, txt_rect)


class Slider:
    """Інтерактивний слайдер зміни числових коефіцієнтів LQR у реальному часі."""

    def __init__(
        self,
        x: int,
        y: int,
        w: int,
        h: int,
        min_val: float,
        max_val: float,
        initial_val: float,
        label: str,
        unit: str = "",
        color: Tuple[int, int, int] = (60, 160, 240),
    ):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = initial_val
        self.label = label
        self.unit = unit
        self.color = color
        self.dragging: bool = False

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            # Збільшена область захвату повзунка
            hit_box = self.rect.inflate(10, 14)
            if hit_box.collidepoint(event.pos):
                self.dragging = True
                self._update_val(event.pos[0])
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self.dragging:
                self.dragging = False
                return True
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self._update_val(event.pos[0])
            return True
        return False

    def _update_val(self, mouse_x: int) -> None:
        norm = (mouse_x - self.rect.x) / float(self.rect.width)
        norm = max(0.0, min(1.0, norm))
        self.val = self.min_val + norm * (self.max_val - self.min_val)

    def draw(
        self,
        surface: pygame.Surface,
        font_lbl: pygame.font.Font,
        font_val: pygame.font.Font,
    ) -> None:
        # Фон треку
        pygame.draw.rect(surface, (35, 42, 55), self.rect, border_radius=4)

        # Заповнена частина
        norm = (self.val - self.min_val) / (self.max_val - self.min_val)
        fill_w = int(self.rect.width * norm)
        if fill_w > 0:
            fill_rect = pygame.Rect(self.rect.x, self.rect.y, fill_w, self.rect.height)
            pygame.draw.rect(surface, self.color, fill_rect, border_radius=4)

        # Рамка треку
        pygame.draw.rect(surface, (70, 85, 110), self.rect, width=1, border_radius=4)

        # Ручка повзунка
        handle_x = self.rect.x + fill_w
        pygame.draw.circle(
            surface,
            (245, 250, 255),
            (handle_x, self.rect.centery),
            self.rect.height // 2 + 3,
        )
        pygame.draw.circle(
            surface,
            self.color,
            (handle_x, self.rect.centery),
            self.rect.height // 2 + 1,
        )

        # Підпис над повзунком
        lbl_surf = font_lbl.render(self.label, True, (190, 205, 225))
        surface.blit(lbl_surf, (self.rect.x, self.rect.y - 18))

        # Значення праворуч над повзунком
        v_str = (
            f"{self.val:.2f}{self.unit}"
            if (self.max_val - self.min_val) < 15.0
            else f"{self.val:.1f}{self.unit}"
        )
        val_surf = font_val.render(v_str, True, (255, 255, 255))
        surface.blit(
            val_surf, (self.rect.right - val_surf.get_width(), self.rect.y - 18)
        )


class Oscilloscope:
    """Осцилограф для моніторингу кута маятника (градуси) та позиції візка (см)."""

    def __init__(self, x: int, y: int, w: int, h: int, max_points: int = 180):
        self.rect = pygame.Rect(x, y, w, h)
        self.max_points = max_points
        self.theta_history: List[float] = []
        self.cart_x_history: List[float] = []

    def push(self, theta_deg: float, cart_x_cm: float) -> None:
        self.theta_history.append(theta_deg)
        self.cart_x_history.append(cart_x_cm)
        if len(self.theta_history) > self.max_points:
            self.theta_history.pop(0)
            self.cart_x_history.pop(0)

    def clear(self) -> None:
        self.theta_history.clear()
        self.cart_x_history.clear()

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        # Фон та рамка
        pygame.draw.rect(surface, (14, 18, 26), self.rect, border_radius=6)
        pygame.draw.rect(surface, (45, 55, 75), self.rect, width=1, border_radius=6)

        # Центральна лінія (0° / 0 см)
        mid_y = self.rect.centery
        pygame.draw.line(
            surface,
            (55, 65, 85),
            (self.rect.x + 5, mid_y),
            (self.rect.right - 5, mid_y),
            1,
        )

        # Графік кута theta (зелений)
        if len(self.theta_history) >= 2:
            pts_th = []
            dx = (self.rect.width - 10) / float(self.max_points - 1)
            for i, val in enumerate(self.theta_history):
                px = int(self.rect.x + 5 + i * dx)
                # Масштаб: ±45° на висоту екрана
                norm_y = val / 45.0
                norm_y = max(-1.0, min(1.0, norm_y))
                py = int(mid_y - norm_y * (self.rect.height / 2.0 - 10))
                pts_th.append((px, py))
            pygame.draw.lines(surface, (70, 220, 130), False, pts_th, 2)

        # Графік візка x (блакитний)
        if len(self.cart_x_history) >= 2:
            pts_x = []
            dx = (self.rect.width - 10) / float(self.max_points - 1)
            for i, val in enumerate(self.cart_x_history):
                px = int(self.rect.x + 5 + i * dx)
                # Масштаб: ±25 см
                norm_y = val / 25.0
                norm_y = max(-1.0, min(1.0, norm_y))
                py = int(mid_y - norm_y * (self.rect.height / 2.0 - 10))
                pts_x.append((px, py))
            pygame.draw.lines(surface, (80, 165, 255), False, pts_x, 1)

        # Легенда
        th_val = self.theta_history[-1] if self.theta_history else 0.0
        x_val = self.cart_x_history[-1] if self.cart_x_history else 0.0

        lbl_th = font.render(f"θ: {th_val:+5.1f}°", True, (70, 220, 130))
        lbl_x = font.render(f"X: {x_val:+5.1f}см", True, (80, 165, 255))
        surface.blit(lbl_th, (self.rect.x + 8, self.rect.y + 4))
        surface.blit(lbl_x, (self.rect.x + 95, self.rect.y + 4))


class ProgressBar:
    """Індикатор відтворення траєкторії MPC."""

    def __init__(self, x: int, y: int, w: int, h: int):
        self.rect = pygame.Rect(x, y, w, h)

    def draw(
        self,
        surface: pygame.Surface,
        progress: float,
        font: pygame.font.Font,
        label: str = "MPC",
    ) -> None:
        pygame.draw.rect(surface, (25, 32, 45), self.rect, border_radius=4)
        prog = max(0.0, min(1.0, progress))
        if prog > 0:
            fill_rect = pygame.Rect(
                self.rect.x, self.rect.y, int(self.rect.width * prog), self.rect.height
            )
            pygame.draw.rect(surface, (190, 80, 230), fill_rect, border_radius=4)
        pygame.draw.rect(surface, (65, 80, 110), self.rect, width=1, border_radius=4)

        txt = font.render(f"{label}: {int(prog * 100)}%", True, (230, 240, 255))
        surface.blit(txt, (self.rect.right + 10, self.rect.y - 2))
