from dash import Dash, html, dcc, Input, Output
import pandas as pd
import plotly.express as px

filename = 'earthnet_database.csv'
df = pd.read_csv(filename,parse_dates=['time'])
df['measure_value::varchar'] = pd.to_numeric(df['measure_value::varchar'])

pd.unique(df['Device'])
coord_data =[]
for device in pd.unique(df['Device']):

    lat = df.where((df['measure_name']=='lat') & (df['Device']==device)).dropna().sort_values(by=['time']).iloc[0]['measure_value::varchar']
    long = df.where((df['measure_name']=='long') & (df['Device']==device)).dropna().sort_values(by=['time']).iloc[0]['measure_value::varchar']
    device_coord = [device, lat, long]
    coord_data.append(device_coord)

coord_df = pd.DataFrame(coord_data, columns=['device', 'lat', "long"])

px.set_mapbox_access_token('pk.eyJ1Ijoic2Fzc3l0aGVzYXNxdWF0Y2giLCJhIjoiY2w2dXJqNmxwMTN5MDNpcTk2NWJxbGN4MyJ9.cAzVjQ1D_8g8FsPIZhoteg')
fig = px.scatter_mapbox(coord_df,
                        lat=coord_df.lat,
                        lon=coord_df.long,
                        hover_name="device",
                        custom_data=['device'],
                        zoom=1)





external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = Dash(__name__, external_stylesheets=external_stylesheets)

app.layout = html.Div([
    html.Div([
        dcc.Graph(
            figure = fig,
            id = 'map',
            hoverData={'points': [{'customdata': 'Device2'}]}
        )
    ]),
   
    html.Div([
        dcc.Graph(id='timeseries')
    ]),

    html.Div([
        dcc.Dropdown(
            df['measure_name'].unique(),
            id = 'parameter_choice',
            value = 'temp'
        )
    ])
])

def create_time_series(dff):

    fig = px.scatter(dff, x='time', y='measure_value::varchar')

    fig.update_traces(mode='lines+markers')

    fig.update_xaxes(showgrid=False)

    return fig

@app.callback(
    Output('timeseries', 'figure'),
    Input('map', 'hoverData'),
    Input('parameter_choice', 'value'))

def update_timeseries(hoverData, chosen_parameter):
    print(hoverData)
    print(hoverData['points'][0]['customdata'][0])
    dff=df[df['Device'] == hoverData['points'][0]['customdata'][0]]
    dff=dff[dff['measure_name'] == chosen_parameter]
    print(dff)
    
    return create_time_series(dff)

if __name__ == '__main__':
    app.run_server(debug=True)

