"""
theme.py — Catppuccin Mocha dark theme for Assembly Grapher.

Apply with:
    from gui.theme import apply, C
    apply(root)          # call once before mainloop
    tk.Text(..., bg=C.SURFACE0, fg=C.TEXT)   # for non-ttk widgets
"""

from __future__ import annotations
import tkinter as tk
from tkinter import ttk


# ── Color palette ─────────────────────────────────────────────────────────────

class C:
    """Catppuccin Mocha palette — use these in panel code for non-ttk widgets."""

    BASE     = "#1e1e2e"   # root window background
    MANTLE   = "#181825"   # status bar / menu
    CRUST    = "#11111b"   # deepest level

    SURFACE0 = "#313244"   # raised panels, treeview background
    SURFACE1 = "#45475a"   # borders, toolbar, treeview headings
    SURFACE2 = "#585b70"   # hovered/active borders

    OVERLAY0 = "#6c7086"   # disabled / muted chrome
    OVERLAY1 = "#7f849c"
    SUBTEXT  = "#a6adc8"   # secondary labels

    TEXT     = "#cdd6f4"   # primary text

    LAVENDER = "#b4befe"   # selection highlight (bg)
    BLUE     = "#89b4fa"   # info
    TEAL     = "#94e2d5"   # teal
    GREEN    = "#a6e3a1"   # success / ok
    YELLOW   = "#f9e2af"   # warning
    PEACH    = "#fab387"   # orange
    RED      = "#f38ba8"   # error
    MAUVE    = "#cba6f7"   # primary accent (buttons, active tabs)
    PINK     = "#f5c2e7"

    # ── Semantic aliases ──────────────────────────────────────────────────────
    SEV_ERROR   = "#f38ba8"
    SEV_WARNING = "#f9e2af"
    SEV_INFO    = "#89b4fa"
    SEV_OK      = "#a6e3a1"

    # Treeview row tints (subtle coloured backgrounds for severity rows)
    ROW_ERROR   = "#3d1f27"
    ROW_WARNING = "#38310e"
    ROW_INFO    = "#1d2d42"
    ROW_OK      = "#1b3028"

    # Tool-type row tints (resources tab)
    ROW_HEX     = "#1e2e2a"
    ROW_SOCKET  = "#1e2030"
    ROW_TORX    = "#2d2318"
    ROW_PHILIPS = "#2d1e20"
    ROW_POZI    = "#291e2d"
    ROW_SLOTTED = "#292920"


# ── Theme application ─────────────────────────────────────────────────────────

def apply(root: tk.Tk) -> None:
    """
    Apply the dark theme to *root* and configure all ttk widget styles.
    Must be called after the root Tk window is created, before mainloop().
    """
    root.configure(bg=C.BASE)

    style = ttk.Style(root)
    style.theme_use("clam")   # most customisable base theme

    # ── Global defaults ───────────────────────────────────────────────────────
    style.configure(".",
        background=C.BASE,
        foreground=C.TEXT,
        bordercolor=C.SURFACE1,
        darkcolor=C.SURFACE0,
        lightcolor=C.SURFACE0,
        troughcolor=C.MANTLE,
        selectbackground=C.LAVENDER,
        selectforeground=C.CRUST,
        insertcolor=C.TEXT,
        relief="flat",
        font=("Segoe UI", 9) if _font_exists("Segoe UI") else ("", 9),
    )

    # ── TFrame ────────────────────────────────────────────────────────────────
    style.configure("TFrame", background=C.BASE)
    style.configure("Toolbar.TFrame", background=C.SURFACE0)
    style.configure("Status.TFrame", background=C.MANTLE)
    style.configure("Card.TFrame", background=C.SURFACE0)

    # ── TLabel ────────────────────────────────────────────────────────────────
    style.configure("TLabel", background=C.BASE, foreground=C.TEXT)
    style.configure("Toolbar.TLabel", background=C.SURFACE0, foreground=C.TEXT)
    style.configure("Status.TLabel", background=C.MANTLE, foreground=C.SUBTEXT,
                    padding=(6, 3))
    style.configure("Heading.TLabel", background=C.BASE,
                    foreground=C.TEXT, font=("", 10, "bold"))
    style.configure("Muted.TLabel", background=C.BASE, foreground=C.SUBTEXT)

    # ── TButton ───────────────────────────────────────────────────────────────
    style.configure("TButton",
        background=C.SURFACE1,
        foreground=C.TEXT,
        bordercolor=C.SURFACE2,
        focusthickness=0,
        relief="flat",
        padding=(10, 4),
    )
    style.map("TButton",
        background=[("active", C.SURFACE2), ("pressed", C.OVERLAY0),
                    ("disabled", C.SURFACE0)],
        foreground=[("disabled", C.OVERLAY0)],
    )

    style.configure("Accent.TButton",
        background=C.MAUVE,
        foreground=C.CRUST,
        padding=(10, 4),
    )
    style.map("Accent.TButton",
        background=[("active", C.LAVENDER), ("pressed", C.BLUE)],
        foreground=[("active", C.CRUST)],
    )

    # ── TNotebook ─────────────────────────────────────────────────────────────
    style.configure("TNotebook",
        background=C.MANTLE,
        bordercolor=C.SURFACE1,
        tabmargins=[2, 5, 2, 0],
    )
    style.configure("TNotebook.Tab",
        background=C.SURFACE0,
        foreground=C.SUBTEXT,
        padding=[14, 6],
        bordercolor=C.SURFACE1,
    )
    style.map("TNotebook.Tab",
        background=[("selected", C.BASE), ("active", C.SURFACE1)],
        foreground=[("selected", C.MAUVE), ("active", C.TEXT)],
        expand=[("selected", [1, 1, 1, 0])],
    )

    # ── Treeview ──────────────────────────────────────────────────────────────
    style.configure("Treeview",
        background=C.SURFACE0,
        foreground=C.TEXT,
        fieldbackground=C.SURFACE0,
        bordercolor=C.SURFACE1,
        rowheight=26,
    )
    style.configure("Treeview.Heading",
        background=C.SURFACE1,
        foreground=C.SUBTEXT,
        bordercolor=C.SURFACE1,
        relief="flat",
        padding=(4, 4),
    )
    style.map("Treeview",
        background=[("selected", C.LAVENDER)],
        foreground=[("selected", C.CRUST)],
    )
    style.map("Treeview.Heading",
        background=[("active", C.SURFACE2)],
        foreground=[("active", C.TEXT)],
    )

    # ── TScrollbar ────────────────────────────────────────────────────────────
    style.configure("TScrollbar",
        background=C.SURFACE1,
        troughcolor=C.MANTLE,
        bordercolor=C.MANTLE,
        arrowcolor=C.OVERLAY0,
        relief="flat",
        arrowsize=10,
    )
    style.map("TScrollbar",
        background=[("active", C.SURFACE2), ("pressed", C.OVERLAY1)],
    )

    # ── TCombobox ─────────────────────────────────────────────────────────────
    style.configure("TCombobox",
        background=C.SURFACE0,
        foreground=C.TEXT,
        fieldbackground=C.SURFACE0,
        selectbackground=C.SURFACE1,
        selectforeground=C.TEXT,
        arrowcolor=C.SUBTEXT,
        bordercolor=C.SURFACE1,
        relief="flat",
        padding=(4, 2),
    )
    style.map("TCombobox",
        background=[("readonly", C.SURFACE0), ("active", C.SURFACE1)],
        fieldbackground=[("readonly", C.SURFACE0)],
        selectbackground=[("readonly", C.SURFACE1)],
        selectforeground=[("readonly", C.TEXT)],
        arrowcolor=[("active", C.TEXT)],
    )
    # Dropdown listbox colours (set globally via option_add)
    root.option_add("*TCombobox*Listbox.background",       C.SURFACE0)
    root.option_add("*TCombobox*Listbox.foreground",       C.TEXT)
    root.option_add("*TCombobox*Listbox.selectBackground", C.LAVENDER)
    root.option_add("*TCombobox*Listbox.selectForeground", C.CRUST)
    root.option_add("*TCombobox*Listbox.relief",           "flat")

    # ── TCheckbutton ──────────────────────────────────────────────────────────
    style.configure("TCheckbutton",
        background=C.BASE,
        foreground=C.TEXT,
        indicatorcolor=C.SURFACE1,
        indicatorrelief="flat",
    )
    style.map("TCheckbutton",
        background=[("active", C.BASE)],
        indicatorcolor=[("selected", C.MAUVE), ("active", C.SURFACE2)],
    )

    # ── TPanedwindow / Sash ───────────────────────────────────────────────────
    style.configure("TPanedwindow", background=C.SURFACE1)
    style.configure("Sash",
        sashthickness=5,
        sashpad=0,
        gripcount=0,
        relief="flat",
        background=C.SURFACE1,
    )

    # ── TSeparator ────────────────────────────────────────────────────────────
    style.configure("TSeparator", background=C.SURFACE1)

    # ── TLabelframe ───────────────────────────────────────────────────────────
    style.configure("TLabelframe",
        background=C.BASE,
        bordercolor=C.SURFACE1,
        relief="solid",
    )
    style.configure("TLabelframe.Label",
        background=C.BASE,
        foreground=C.SUBTEXT,
        font=("", 9, "bold"),
    )

    # ── Menu (tk.Menu — not ttk, needs direct configure) ─────────────────────
    # Caller must pass root.option_add for menus or configure manually.
    root.option_add("*Menu.background",       C.MANTLE)
    root.option_add("*Menu.foreground",       C.TEXT)
    root.option_add("*Menu.activeBackground", C.SURFACE1)
    root.option_add("*Menu.activeForeground", C.TEXT)
    root.option_add("*Menu.relief",           "flat")
    root.option_add("*Menu.borderWidth",      0)


# ── Helper ────────────────────────────────────────────────────────────────────

def _font_exists(name: str) -> bool:
    """Return True if a named font is available on this system."""
    try:
        import tkinter.font as tkfont
        return name in tkfont.families()
    except Exception:
        return False


def style_text_widget(widget: tk.Text, **extra) -> None:
    """Configure a tk.Text widget with dark-theme defaults."""
    widget.configure(
        bg=C.SURFACE0,
        fg=C.TEXT,
        insertbackground=C.TEXT,
        selectbackground=C.LAVENDER,
        selectforeground=C.CRUST,
        relief=tk.FLAT,
        highlightthickness=0,
        borderwidth=0,
        **extra,
    )
