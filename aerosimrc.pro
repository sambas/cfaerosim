TEMPLATE  = subdirs

win32 {
    SUBDIRS += plugin
}

SUBDIRS += qt-solutions/qtwinmigrate

plugin.file   = src/plugin.pro
qtwinmigrate.file  = qt-solutions/qtwinmigrate/qtwinmigrate.pro
