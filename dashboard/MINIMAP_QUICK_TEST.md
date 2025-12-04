# 🧪 Quick MiniMap Test

## ✅ Configuration Applied

The MiniMap is now configured exactly as specified:

```javascript
miniMap = new L.Control.MiniMap(miniMapLayer, {
    toggleDisplay: true,      // ✅ Toggle button enabled
    minimized: false,         // ✅ Start expanded
    position: 'bottomright',  // ✅ Bottom-right corner
    width: 150,              // ✅ 150px width
    height: 150,             // ✅ 150px height
    zoomLevelOffset: -5       // ✅ 5 zoom levels more zoomed out
}).addTo(mainMap);
```

## 🧪 Quick Test

1. **Open dashboard**: http://localhost:8000/dashboard.html
2. **Look for MiniMap**: Should appear in bottom-right corner (150×150px)
3. **Pan main map**: Drag map around
   - ✅ MiniMap viewport rectangle should move
4. **Zoom main map**: Use scroll or +/- buttons
   - ✅ MiniMap viewport rectangle should resize
   - ✅ MiniMap should show more zoomed-out view
5. **Click toggle**: Click minimize button on MiniMap
   - ✅ Should collapse/expand

## ✅ Expected Behavior

- **Viewport Rectangle**: Green rectangle shows current main map area
- **Auto-sync**: Rectangle updates automatically when main map changes
- **Zoom Offset**: MiniMap shows 5 zoom levels more zoomed out
- **Toggle**: Button in top-left of MiniMap to hide/show

## 🐛 If MiniMap Doesn't Appear

1. Check browser console (F12) for errors
2. Verify CDN loaded: Look for "MiniMap initialized" in console
3. Check if Leaflet loaded first (MiniMap depends on Leaflet)

The MiniMap should automatically sync with the main map - no additional code needed!

