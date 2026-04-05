Set sh  = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
dir     = Left(WScript.ScriptFullName, InStrRev(WScript.ScriptFullName, "\"))
script  = dir & "gui_main.py"
logFile = dir & "vbs_launcher.log"
user    = sh.ExpandEnvironmentStrings("%USERPROFILE%")
local   = sh.ExpandEnvironmentStrings("%LOCALAPPDATA%")

' ── logging helper ─────────────────────────────────────────────────────────
Sub Log(msg)
    Dim f
    Set f = fso.OpenTextFile(logFile, 8, True)   ' 8 = append
    f.WriteLine Now & "  " & msg
    f.Close
End Sub

Log "=== AssemblyGrapher launcher started ==="
Log "Script: " & script

' ── helper: run python -c "..." and return exit code ───────────────────────
Function HasOCP(pyExe)
    On Error Resume Next
    Dim ret
    ret = sh.Run("""" & pyExe & """ -c ""import OCP.STEPControl""", 0, True)
    If Err.Number <> 0 Then ret = 1
    On Error GoTo 0
    HasOCP = (ret = 0)
End Function

' ── build ordered list of python.exe / pythonw.exe candidates ──────────────
' Prefer pythonw (no console flicker) but also keep python as backup.
' Static list — conda candidates first so existing environments win,
' plain Python installs last.

Dim allPy(49)
Dim nPy : nPy = 0

Sub AddPy(p)
    If fso.FileExists(p) Then
        allPy(nPy) = p
        nPy = nPy + 1
    End If
End Sub

' ── 1a. Ask each conda.exe for its base path ───────────────────────────────
Dim condaExes(11)
condaExes(0)  = user  & "\miniforge3\Scripts\conda.exe"
condaExes(1)  = user  & "\mambaforge\Scripts\conda.exe"
condaExes(2)  = user  & "\miniconda3\Scripts\conda.exe"
condaExes(3)  = user  & "\anaconda3\Scripts\conda.exe"
condaExes(4)  = local & "\miniforge3\Scripts\conda.exe"
condaExes(5)  = local & "\mambaforge\Scripts\conda.exe"
condaExes(6)  = local & "\miniconda3\Scripts\conda.exe"
condaExes(7)  = local & "\anaconda3\Scripts\conda.exe"
condaExes(8)  = "C:\miniforge3\Scripts\conda.exe"
condaExes(9)  = "C:\miniconda3\Scripts\conda.exe"
condaExes(10) = "C:\ProgramData\miniforge3\Scripts\conda.exe"
condaExes(11) = "C:\ProgramData\miniconda3\Scripts\conda.exe"

Dim ci
For ci = 0 To 11
    If fso.FileExists(condaExes(ci)) Then
        Log "Found conda: " & condaExes(ci)
        On Error Resume Next
        Dim oExec : Set oExec = sh.Exec("""" & condaExes(ci) & """ info --base")
        If Err.Number = 0 Then
            Dim base : base = ""
            Do While Not oExec.StdOut.AtEndOfStream
                Dim ln : ln = Trim(oExec.StdOut.ReadLine())
                If ln <> "" Then base = ln
            Loop
            If base <> "" Then
                Log "  conda base: " & base
                AddPy base & "\pythonw.exe"
                AddPy base & "\python.exe"
            End If
        End If
        On Error GoTo 0
    End If
Next

' ── 1b. Static fallback paths (plain Python installs) ──────────────────────
AddPy local & "\Programs\Python\Python313\pythonw.exe"
AddPy local & "\Programs\Python\Python313\python.exe"
AddPy local & "\Programs\Python\Python312\pythonw.exe"
AddPy local & "\Programs\Python\Python312\python.exe"
AddPy local & "\Programs\Python\Python311\pythonw.exe"
AddPy local & "\Programs\Python\Python311\python.exe"
AddPy local & "\Programs\Python\Python310\pythonw.exe"
AddPy local & "\Programs\Python\Python310\python.exe"
AddPy "C:\Python313\pythonw.exe"
AddPy "C:\Python312\pythonw.exe"
AddPy "C:\Python311\pythonw.exe"
AddPy "C:\Python310\pythonw.exe"

Log "Total candidates found: " & nPy

' ── 2. Pick the FIRST candidate that has OCP (cadquery-ocp) installed ───────
Dim chosen  : chosen  = ""
Dim fallback: fallback = ""

Dim k
For k = 0 To nPy - 1
    Dim py : py = allPy(k)
    Log "Testing: " & py
    If HasOCP(py) Then
        Log "  -> OCP OK — using this Python"
        chosen = py
        Exit For
    Else
        Log "  -> OCP not found"
        If fallback = "" Then fallback = py   ' keep first available as last resort
    End If
Next

' ── 3. Fall back if no Python has OCP ──────────────────────────────────────
If chosen = "" Then
    If fallback <> "" Then
        chosen = fallback
        Log "WARNING: no Python with OCP found — falling back to " & fallback
        Log "  STEP import will not work. Run: pip install cadquery"
    Else
        ' Last resort: use Windows py launcher
        Log "WARNING: no Python found at all — using 'py' launcher"
    End If
End If

' ── 4. Launch ───────────────────────────────────────────────────────────────
Log "Launching: " & chosen & " " & script
If chosen <> "" Then
    sh.Run """" & chosen & """ """ & script & """", 0, False
Else
    sh.Run "py """ & script & """", 0, False
End If
