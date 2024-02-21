#define grafico_page "<!DOCTYPE html>\
<html>\
<head>\
    <meta http-equiv='Content-Type' content='text/html; charset=UTF-8' />\
    <style type='text/css'>\
        body {\
            margin: 0;\
            background-image: url('https://cdn.discordapp.com/attachments/1065034859845791848/1208186413238059119/main.png?ex=65e25e47&is=65cfe947&hm=d3da2b154bf620a6e5ab9b2068df7bd01a0d768e7552a249c5d5b017e0f4d5cf&');\
            background-size: cover;\
        }\
        canvas {\
            display: block;\
            margin: 40px auto;\
        }\
\
        #NavBar {\
            display: block;\
            background: url('https://cdn.discordapp.com/attachments/1065034859845791848/1208182477475155999/image.png?ex=65e25a9d&is=65cfe59d&hm=4e8665e50ca38e426c7613facf101243d2d2d22f544bd6823b7830fa30d393f7&');\
            background-size: auto;\
            background-position: center;\
            background-repeat: no-repeat;\
            width: 100%;\
            height: 60px;\
        }\
    </style>\
</head>\
<body>\
    <div id='NavBar'>Hola Como estas?</div>\
    <canvas id='my-canvas'\
        style='width:100%;max-width:1000px;background:#021d38;border:1px solid white; border-radius: 10px;'></canvas>\
    <!-- <script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.9.4/Chart.js'> -->\
    <script src='http://localhost:3001/chart.js'></script>\
    </script>\
    <script type='text/javascript' charset='utf-8'>\
        const xValues = [];\
        const yValues = [];\
        for (let i = 0; i < 50; i++) {\
            xValues.push(0);\
            yValues.push(0);\
        }\
        var whiteColorGraph = '#FFFFFF'\
        var gridColor = '#042e59'\
        var chart = new Chart('my-canvas', {\
            type: 'line',\
            data: {\
                labels: xValues,\
                datasets: [{\
                    label: 'Voltaje',\
                    data: yValues,\
                    fill: false,\
                    tension: 0.1,\
                    pointRadius: 4,\
                    borderColor: '#0094c0',\
                    backgroundColor: '#00D2FF',\
                    pointBackgroundColor: '#00D2FF',\
                    fontColor: 'white',\
                    cubicInterpolationMode: 'monotone',\
                    tension: 0.4\
                }, {\
                    label: 'Corriente',\
                    data: yValues,\
                    fill: false,\
                    tension: 0.1,\
                    pointRadius: 4,\
                    borderColor: '#009677',\
                    backgroundColor: '#38D2B0',\
                    pointBackgroundColor: '#38D2B0',\
                    cubicInterpolationMode: 'monotone',\
                    tension: 0.4\
                }]\
            },\
            options: {\
                scales: {\
                    yAxes: [{\
                        ticks: {\
                            beginAtZero: true,\
                            fontColor: whiteColorGraph,\
                            stepSize: 10,\
                            suggestedMin: 0,\
                            suggestedMax: 50\
                        },\
                        gridLines: {\
                            color: gridColor, // Setting grid color for y-axis here\
                        }\
                    }],\
                    xAxes: [{\
                        ticks: {\
                            beginAtZero: true,\
                            fontColor: whiteColorGraph,\
                        },\
                        gridLines: {\
                            color: gridColor, // Setting grid color for y-axis here\
                        }\
                    }]\
                },\
                title: {\
                    display: true,\
                    text: 'Carga electrónica UTN FRA',\
                    fontColor: whiteColorGraph\
                }\
            },\
        });\
        //Add point function\
        function addPoint() {\
            fetch('/data.cgi').then(response => response.json()).then(({ x, y }) => {\
                const date = new Date();\
                if (xValues.length >= 50)\
                    xValues.shift();\
                if (yValues.length >= 50)\
                    yValues.shift()\
                xValues.push(`${date.getHours()}:${date.getMinutes()}:${date.getSeconds()}`);\
                yValues.push(y);\
                chart.update();\
                console.log('ready');\
            }).catch(() => console.log('Error from api'))\
        }\
        //Cada 1 seg agregar un punto proveniente de la api\
        let identificadorIntervaloDeTiempo = setInterval(() => addPoint(), 1000);\
    </script>\
</body>\
</html>"