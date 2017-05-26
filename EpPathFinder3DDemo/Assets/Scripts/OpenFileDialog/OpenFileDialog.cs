using System;
using System.Runtime.InteropServices;
using UnityEngine.UI;

[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto)]

public class OpenFileName {
	public int      structSize = 0;
	public IntPtr   dlgOwner = IntPtr.Zero;
	public IntPtr   instance = IntPtr.Zero;
	public String   filter = null;
	public String   customFilter = null;
	public int      maxCustFilter = 0;
	public int      filterIndex = 0;
	public String   file = null;
	public int      maxFile = 0;
	public String   fileTitle = null;
	public int      maxFileTitle = 0;
	public String   initialDir = null;
	public String   title = null;
	public int      flags = 0;
	public short    fileOffset = 0;
	public short    fileExtension = 0;
	public String   defExt = null;
	public IntPtr   custData = IntPtr.Zero;
	public IntPtr   hook = IntPtr.Zero;
	public String   templateName = null;
	public IntPtr   reservedPtr = IntPtr.Zero;
	public int      reservedInt = 0;
	public int      flagsEx = 0;

	public static bool Open(string title, Action<string> handler, string defExt ="", string filter = "All Files\0*.*\0\0", string initialDir = null){
		OpenFileName ofn = new OpenFileName();
		ofn.structSize = Marshal.SizeOf(ofn);
		ofn.filter = filter;
		ofn.file = new string(new char[256]);
		ofn.maxFile = ofn.file.Length;
		ofn.fileTitle = new string(new char[64]);
		ofn.maxFileTitle = ofn.fileTitle.Length;
		if (initialDir == null) {
			initialDir = UnityEngine.Application.dataPath;
		}
		ofn.initialDir = initialDir;
		ofn.title = title;
		ofn.defExt = defExt;
		ofn.flags = 0x00080000 | 0x00001000 | 0x00000800 | 0x00000200 | 0x00000008;//OFN_EXPLORER|OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST| OFN_ALLOWMULTISELECT|OFN_NOCHANGEDIR
		if(OpenFileDialogDLL.GetOpenFileName(ofn)) {
			handler (ofn.file);
			return true;
		}
		return false;
	}
}

public class OpenFileDialogDLL {
	[DllImport("Comdlg32.dll", SetLastError = true, ThrowOnUnmappableChar = true, CharSet = CharSet.Auto)]
	public static extern bool GetOpenFileName([In, Out] OpenFileName ofn);
	public static bool GetOpenFileName1([In, Out] OpenFileName ofn) {
		return GetOpenFileName(ofn);
	}
}