{
  "startPoint": {
    "x": 26.5,
    "y": 130,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180
  },
  "lines": [
    {
      "name": "LaunchClose1",
      "endPoint": {
        "x": 36,
        "y": 107,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 134,
        "degrees": 134
      },
      "controlPoints": [],
      "color": "#98AAA7",
      "id": "line-0ncqthfk4b49",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "name": "ArtifactsSet1",
      "endPoint": {
        "x": 23.75,
        "y": 83.8,
        "heading": "constant",
        "startDeg": 134,
        "endDeg": 270,
        "degrees": 270,
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 24,
          "y": 113
        }
      ],
      "color": "#98AAA7",
      "id": "line-61071kht4bd",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpalvzfi-nrfkf0",
      "endPoint": {
        "x": 15.25,
        "y": 72,
        "heading": "linear",
        "reverse": false,
        "startDeg": 270,
        "endDeg": 270
      },
      "controlPoints": [
        {
          "x": 24.052863910422055,
          "y": 72.1714900947459
        }
      ],
      "color": "#98AAA7",
      "name": "Open Gate",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "name": "LaunchClose2",
      "endPoint": {
        "x": 36,
        "y": 107,
        "heading": "linear",
        "reverse": true,
        "startDeg": 270,
        "endDeg": 134,
        "degrees": 35
      },
      "controlPoints": [
        {
          "x": 50.46608527131784,
          "y": 72.39018087855298
        }
      ],
      "color": "#98AAA7",
      "id": "line-uzsyiqkntn",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "name": "ArtifactsSet2",
      "endPoint": {
        "x": 24,
        "y": 61,
        "heading": "constant",
        "reverse": false,
        "startDeg": 134,
        "endDeg": 134,
        "degrees": 270
      },
      "controlPoints": [
        {
          "x": 24,
          "y": 100
        }
      ],
      "color": "#98AAA7",
      "id": "line-50l0ot5q3tp",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpam1w02-6wjckg",
      "endPoint": {
        "x": 15.25,
        "y": 72,
        "heading": "constant",
        "reverse": true,
        "startDeg": 0,
        "endDeg": 0,
        "degrees": 270
      },
      "controlPoints": [
        {
          "x": 21.625,
          "y": 66.5
        }
      ],
      "color": "#98AAA7",
      "name": "Open Gate",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "name": "LaunchClose3",
      "endPoint": {
        "x": 36,
        "y": 107,
        "heading": "linear",
        "reverse": false,
        "startDeg": 270,
        "endDeg": 134,
        "degrees": 0
      },
      "controlPoints": [
        {
          "x": 50.5,
          "y": 72
        }
      ],
      "color": "#98AAA7",
      "id": "line-cj4mt2rfbc",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-0ncqthfk4b49"
    },
    {
      "kind": "path",
      "lineId": "line-61071kht4bd"
    },
    {
      "kind": "path",
      "lineId": "mpalvzfi-nrfkf0"
    },
    {
      "kind": "path",
      "lineId": "line-uzsyiqkntn"
    },
    {
      "kind": "path",
      "lineId": "line-50l0ot5q3tp"
    },
    {
      "kind": "path",
      "lineId": "mpam1w02-6wjckg"
    },
    {
      "kind": "path",
      "lineId": "line-cj4mt2rfbc"
    }
  ],
  "pathChains": [
    {
      "id": "chain-mpaiydcr-nf720w",
      "name": "Main Chain",
      "color": "#98AAA7",
      "lineIds": [
        "line-0ncqthfk4b49",
        "line-61071kht4bd",
        "line-uzsyiqkntn",
        "line-50l0ot5q3tp",
        "line-cj4mt2rfbc",
        "mpalvzfi-nrfkf0",
        "mpam1w02-6wjckg"
      ]
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 6,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false,
    "showHeadingArrow": false,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1
  },
  "version": "1.2.1",
  "timestamp": "2026-05-18T20:38:20.692Z"
}