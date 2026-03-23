Set sh  = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
dir     = Left(WScript.ScriptFullName, InStrRev(WScript.ScriptFullName, "\"))
script  = dir & "gui_main.py"
user    = sh.ExpandEnvironmentStrings("%USERPROFILE%")
local   = sh.ExpandEnvironmentStrings("%LOCALAPPDATA%")

Dim chosen : chosen = ""

' ── 1. Ask conda where its base environment lives ─────────────────────────
' This works regardless of install path (miniconda, anaconda, miniforge, etc.)
On Error Resume Next
Dim oExec
Set oExec = sh.Exec("conda info --base")
If Err.Number = 0 Then
    Dim condaBase : condaBase = ""
    ' Read all output lines to get the last non-empty line
    Do While Not oExec.StdOut.AtEndOfStream
        Dim ln : ln = Trim(oExec.StdOut.ReadLine())
        If ln <> "" Then condaBase = ln
    Loop
    ' Try pythonw.exe first (no console), then python.exe (hidden via Run 0)
    If condaBase <> "" Then
        If fso.FileExists(condaBase & "\pythonw.exe") Then
            chosen = condaBase & "\pythonw.exe"
        ElseIf fso.FileExists(condaBase & "\python.exe") Then
            chosen = condaBase & "\python.exe"
        End If
    End If
End If
On Error GoTo 0

' ── 2. Static path scan fallback (conda variants → standard Python) ────────
If chosen = "" Then
    Dim cands(25)
    ' Common conda base locations
    cands(0)  = user  & "\miniconda3\pythonw.exe"
    cands(1)  = user  & "\miniconda3\python.exe"
    cands(2)  = user  & "\anaconda3\pythonw.exe"
    cands(3)  = user  & "\anaconda3\python.exe"
    cands(4)  = user  & "\miniforge3\pythonw.exe"
    cands(5)  = user  & "\miniforge3\python.exe"
    cands(6)  = user  & "\mambaforge\pythonw.exe"
    cands(7)  = user  & "\mambaforge\python.exe"
    cands(8)  = local & "\miniconda3\pythonw.exe"
    cands(9)  = local & "\miniconda3\python.exe"
    cands(10) = local & "\anaconda3\pythonw.exe"
    cands(11) = local & "\anaconda3\python.exe"
    cands(12) = local & "\miniforge3\pythonw.exe"
    cands(13) = local & "\miniforge3\python.exe"
    cands(14) = local & "\mambaforge\pythonw.exe"
    cands(15) = local & "\mambaforge\python.exe"
    cands(16) = "C:\miniconda3\pythonw.exe"
    cands(17) = "C:\anaconda3\pythonw.exe"
    cands(18) = "C:\miniforge3\pythonw.exe"
    cands(19) = "C:\ProgramData\miniconda3\pythonw.exe"
    cands(20) = "C:\ProgramData\anaconda3\pythonw.exe"
    ' Standard Python installer (AppData) — last resort
    cands(21) = local & "\Programs\Python\Python313\pythonw.exe"
    cands(22) = local & "\Programs\Python\Python312\pythonw.exe"
    cands(23) = local & "\Programs\Python\Python311\pythonw.exe"
    cands(24) = local & "\Programs\Python\Python310\pythonw.exe"
    cands(25) = local & "\Programs\Python\Python39\pythonw.exe"

    Dim i
    For i = 0 To 25
        If fso.FileExists(cands(i)) Then
            chosen = cands(i)
            Exit For
        End If
    Next
End If

' ── 3. Launch ──────────────────────────────────────────────────────────────
If chosen <> "" Then
    ' Window style 0 = hidden (no console), even when using python.exe
    sh.Run """" & chosen & """ """ & script & """", 0, False
Else
    ' Last resort: py launcher (C:\Windows\py.exe)
    sh.Run "py """ & script & """", 0, False
End If
