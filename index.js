const express = require('express')
const { spawn } = require('child_process')
const app = express()
const port = 3030

app.use(express.json())

app.post('/', (req, res) => {

    let body = req.body

    // Fetch solver
    var solver = ""
    if (body.hasOwnProperty('options')) {
        if (body.options.hasOwnProperty('solver')) {
            solver = body.options.solver
        }
    }
    delete body.options

    let data = JSON.stringify(body)
    var largeDataSet = [];

    // spawn new child process to call the python script
    const python = spawn('python', [solver + '.py', data]);

    // collect data from script
    python.stdout.on('data', function (data) {
        largeDataSet.push(data)
    });

    // in close event we are sure that stream from child process is closed
    python.on('close', (code) => {
        console.log(`child process close all stdio with code ${code}`);
        // send data to browser
        res.send(largeDataSet.join(""))
    });

})

app.listen(port, () => console.log(`OrTools HTTP Wrapper listening on port ${port}!`))