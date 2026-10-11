"""
ui.py
=====
Модуль графічних компонентів Pygame: адаптивні кнопки, слайдери,
двоканальний осцилограф кутів та індикатор прогресу MPC.
Повністю підтримує динамічне масштабування (DPI-aware) під будь-яку роздільну здатність екрана (Full HD, 2K, 4K).
"""

from typing import Callable, Tuple, List, Optional
import pygame
import numpy as np


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
        on_release: Optional[Callable[[], None]] = None
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
        bg = self.hover_color if self.hovered else self.color
        pygame.draw.rect(surface, bg, self.rect, border_radius=self.border_radius)
        pygame.draw.rect(surface, (255, 255, 255, 50), self.rect, width=1, border_radius=self.border_radius)

        txt_surf = font.render(self.text, True, (240, 245, 255))
        txt_rect = txt_surf.get_rect(center=self.rect.center)
        surface.blit(txt_surf, txt_rect)


class Slider:
    """Адаптивний горизонтальний повзунок для регулювання коефіцієнтів."""

    def __init__(
        self,
        x: int,
        y: int,
        w: int,
        h: int,
        min_val: float,
        max_val: float,
        init_val: float,
        label: str,
        unit: str = "",
        color: Tuple[int, int, int] = (60, 130, 220),
        on_change: Optional[Callable[[float], None]] = None
    ):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = init_val
        self.label = label
        self.unit = unit
        self.color = color
        self.on_change = on_change
        self.dragging: bool = False

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            hitbox = self.rect.inflate(18, 20)
            if hitbox.collidepoint(event.pos):
                self.dragging = True
                self._update_val(event.pos[0])
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self._update_val(event.pos[0])
            return True
        return False

    def _update_val(self, mouse_x: int) -> None:
        rel = (mouse_x - self.rect.x) / max(1, self.rect.w)
        rel = max(0.0, min(1.0, rel))
        self.val = self.min_val + rel * (self.max_val - self.min_val)
        if self.on_change:
            self.on_change(self.val)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        # Фон треку
        pygame.draw.rect(surface, (34, 40, 54), self.rect, border_radius=4)

        # Заповнення активної частини
        rel = (self.val - self.min_val) / max(1e-5, (self.max_val - self.min_val))
        fill_w = max(4, int(rel * self.rect.w))
        fill_rect = pygame.Rect(self.rect.x, self.rect.y, fill_w, self.rect.h)
        pygame.draw.rect(surface, self.color, fill_rect, border_radius=4)

        # Ручка повзунка (пропорційна висоті треку)
        handle_w = max(10, int(self.rect.h * 0.95))
        handle_h = self.rect.h + 8
        handle_x = self.rect.x + fill_w
        handle_rect = pygame.Rect(handle_x - handle_w // 2, self.rect.y - 4, handle_w, handle_h)
        
        h_color = (255, 255, 255) if self.dragging else (210, 225, 245)
        pygame.draw.rect(surface, h_color, handle_rect, border_radius=3)
        pygame.draw.rect(surface, (20, 25, 35), handle_rect, width=1, border_radius=3)

        # Текст над повзунком з динамічним відступом
        txt = font.render(f"{self.label}: {self.val:.1f}{self.unit}", True, (215, 225, 240))
        label_y = self.rect.y - font.get_linesize() - 3
        surface.blit(txt, (self.rect.x, label_y))


class Oscilloscope:
    """Адаптивний осцилограф для відображення динаміки кутів theta1(t) та theta2(t)."""

    def __init__(self, x: int, y: int, w: int, h: int, max_points: int = 180):
        self.rect = pygame.Rect(x, y, w, h)
        self.max_points = max_points
        self.history_th1: List[float] = []
        self.history_th2: List[float] = []

    def add_sample(self, th1_deg: float, th2_deg: float) -> None:
        self.history_th1.append(th1_deg)
        self.history_th2.append(th2_deg)
        if len(self.history_th1) > self.max_points:
            self.history_th1.pop(0)
            self.history_th2.pop(0)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        pygame.draw.rect(surface, (20, 25, 36), self.rect, border_radius=8)
        pygame.draw.rect(surface, (45, 55, 75), self.rect, width=1, border_radius=8)

        mid_y = self.rect.y + self.rect.h // 2
        
        # Зона стійкості (±15°)
        band_h = int((15.0 / 60.0) * (self.rect.h // 2))
        band_rect = pygame.Rect(self.rect.x + 1, mid_y - band_h, self.rect.w - 2, band_h * 2)
        pygame.draw.rect(surface, (28, 55, 40), band_rect)

        # Центральна лінія (0°)
        pygame.draw.line(surface, (60, 72, 95), (self.rect.x, mid_y), (self.rect.x + self.rect.w, mid_y), 1)

        # Заголовок осцилографа
        lbl = font.render("Осцилограф кутів θ1 (син) / θ2 (помар)", True, (160, 175, 200))
        surface.blit(lbl, (self.rect.x + 10, self.rect.y + 6))

        if len(self.history_th1) >= 2:
            dx = self.rect.w / float(self.max_points)
            scale_y = (self.rect.h // 2 - 10) / 60.0  # макс діапазон 60°

            pts1 = []
            pts2 = []
            for i in range(len(self.history_th1)):
                px = self.rect.x + int(i * dx)
                py1 = mid_y - int(self.history_th1[i] * scale_y)
                py2 = mid_y - int(self.history_th2[i] * scale_y)

                py1 = max(self.rect.y + 3, min(self.rect.y + self.rect.h - 3, py1))
                py2 = max(self.rect.y + 3, min(self.rect.y + self.rect.h - 3, py2))

                pts1.append((px, py1))
                pts2.append((px, py2))

            pygame.draw.lines(surface, (60, 160, 240), False, pts1, 2)
            pygame.draw.lines(surface, (245, 140, 50), False, pts2, 2)


class ProgressBar:
    """Адаптивна смуга прогресу виконання маневру MPC."""

    def __init__(self, x: int, y: int, w: int, h: int):
        self.rect = pygame.Rect(x, y, w, h)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, progress: float) -> None:
        pygame.draw.rect(surface, (26, 32, 44), self.rect, border_radius=5)
        
        fill_w = int(max(0.0, min(1.0, progress)) * self.rect.w)
        if fill_w > 0:
            fill_rect = pygame.Rect(self.rect.x, self.rect.y, fill_w, self.rect.h)
            pygame.draw.rect(surface, (45, 185, 230), fill_rect, border_radius=5)
            
        pygame.draw.rect(surface, (60, 75, 105), self.rect, width=1, border_radius=5)

        pct_val = int(progress * 100)
        pct_text = f"Прогрес MPC Swing-Up: {pct_val}%"
        txt = font.render(pct_text, True, (215, 235, 255))
        label_y = self.rect.y - font.get_linesize() - 3
        surface.blit(txt, (self.rect.x, label_y))
